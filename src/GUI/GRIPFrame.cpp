/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <fstream>

#include "GUI.h"
#include <Tools/Collision.h>
#include "GRIPFrame.h"
#include "Viewer.h"
#include "TreeView.h"

#include <Tabs/AllTabs.h>
#include <Tabs/GRIPTab.h>
#include "GRIPSlider.h"
#include "GRIPTimeSlice.h"
#include "GRIPFrame.h"
#include <Tools/utils.h>

#include "icons/open.xpm"
#include "icons/save.xpm"
#include "icons/redo.xpm"
#include "icons/up.xpm"
#include "icons/anchor.xpm"
#include "icons/asterisk.xpm"
#include "icons/camera.xpm"
#include "icons/film.xpm"
#include "icons/clock.xpm"

#define ID_TOOLBAR 1257
#define ID_TIMESLIDER 1258

enum toolNums {
	Tool_open = 1262,
	Tool_save = 1263,
	Tool_quickload = 1264,

	Tool_linkorder = 1265,
	Tool_checkcollisions = 1266,
	Tool_screenshot = 1267,
	Tool_movie = 1268
};

extern bool check_for_collisions;

//wxSTD_MDIPARENTFRAME ICON wxICON(ROBOT_xpm)

/**
 * @function GRIPFrame
 * @brief Constructor 
 * @date 2011-10-13
 */
GRIPFrame::GRIPFrame(const wxString& title) :
	wxFrame(NULL, wxID_ANY, title) {

	tPrecision = 1000;
	tMax = 5;
	tMax = 0;
	InitTimer("", 0);
	std::cout << "GRIPFrame 1" << std::endl;
	wxMenu *fileMenu = new wxMenu;
	wxMenu *helpMenu = new wxMenu;
	wxMenu *settingsMenu = new wxMenu;
	wxMenu *bgMenu = new wxMenu;
	//wxMenu *saveMenu = new wxMenu;

	fileMenu->Append(MenuLoad, wxT("L&oad\tAlt-O"));
	fileMenu->Append(MenuQuickLoad, wxT("Q&uickLoad\tAlt-Shift-Q"));
	fileMenu->Append(MenuSaveScene, wxT("Save Scene"));
	//saveMenu->Append(MenuSaveRobot, wxT("Save Robot"));
	//fileMenu->AppendSubMenu(saveMenu, wxT("S&ave\tAlt-S"));
	fileMenu->Append(MenuClose, wxT("C&lose\tAlt-C"));
	fileMenu->AppendSeparator();
	fileMenu->Append(MenuQuit, wxT("E&xit\tAlt-Q"));

	bgMenu->Append(MenuBgWhite, wxT("White"));
	bgMenu->Append(MenuBgBlack, wxT("Black"));

	settingsMenu->AppendSubMenu(bgMenu, wxT("Background"));
	settingsMenu->Append(MenuCameraReset, wxT("Reset Camera"));

	helpMenu->Append(MenuAbout, wxT("&About...\tF1"), wxT("Show about dialog"));

	wxMenu *palletMenu = new wxMenu;
	palletMenu->Append(MenuPalletLoad, wxT("L&oad Palletizing Files"));

	wxMenuBar *menuBar = new wxMenuBar();
	menuBar->Append(fileMenu, wxT("&File"));
	menuBar->Append(settingsMenu, wxT("&Settings"));
	menuBar->Append(helpMenu, wxT("&Help"));
	menuBar->Append(palletMenu, wxT("&Palletizing"));
	SetMenuBar(menuBar);

	toolBarBitmaps[0] = wxIcon(open_xpm);
	toolBarBitmaps[1] = wxIcon(save_xpm);
	toolBarBitmaps[2] = wxIcon(redo_xpm);
	toolBarBitmaps[3] = wxIcon(anchor_xpm);
	toolBarBitmaps[4] = wxIcon(asterisk_xpm);
	toolBarBitmaps[5] = wxIcon(camera_xpm);
	toolBarBitmaps[6] = wxIcon(film_xpm);

	wxBitmap clockBmp = wxBitmap(clock_xpm);

	filebar = new wxToolBar(this, ID_TOOLBAR, wxPoint(0, 0),
			wxSize(prefTreeViewWidth + 50, toolBarHeight), wxTB_HORIZONTAL);

	filebar->SetToolBitmapSize(wxSize(16, 16));
	filebar->AddTool(wxID_OPEN, _T("Open"), toolBarBitmaps[0],
			toolBarBitmaps[0], wxITEM_NORMAL, _T("Open .rscene file (Alt-O)"));
	filebar->AddTool(wxID_SAVE, _T("Save"), toolBarBitmaps[1],
			toolBarBitmaps[1], wxITEM_NORMAL,
			_T("Save world to .rscene file (Alt-S)"));
	filebar->AddSeparator();
	filebar->AddTool(Tool_quickload, _T("Quick Load"), toolBarBitmaps[2],
			toolBarBitmaps[2], wxITEM_NORMAL,
			_T("Load last viewed scene (Alt-Shift-Q)"));
	filebar->AddSeparator();
	filebar->AddTool(Tool_screenshot, _T("Screenshot"), toolBarBitmaps[5],
			toolBarBitmaps[5], wxITEM_NORMAL, _T("Export screenshot"));
	filebar->AddTool(Tool_movie, _T("Movie"), toolBarBitmaps[6],
			toolBarBitmaps[6], wxITEM_NORMAL, _T("Export film sequence"));

	//timeSlider = new GRIPSlider(clockBmp,0,1000,100,0,100,500,this,ID_TIMESLIDER,true);
	wxPanel* timePanel = new wxPanel(this, -1, wxDefaultPosition,
			wxDefaultSize, 0);

#ifdef WIN32 /// For windows use a thicker slider - it looks nice
	timeTrack = new wxSlider(timePanel,1009,0,0,1000,wxDefaultPosition, wxSize(30,100), wxSL_BOTH | wxSL_VERTICAL | wxALIGN_CENTRE);
#else
	timeTrack = new wxSlider(timePanel, 1009, 0, 0, 1000, wxDefaultPosition,
			wxSize(16, 100), wxSL_BOTH | wxSL_VERTICAL | wxALIGN_CENTRE);
#endif
	wxStaticBitmap *timeButton = new wxStaticBitmap(timePanel, -1, clockBmp,
			wxDefaultPosition, wxSize(16, 16), wxALIGN_CENTRE);
	wxSizer *sizerTime = new wxBoxSizer(wxVERTICAL);
	sizerTime->Add(timeButton, 0, wxALIGN_CENTRE | wxEXPAND | wxALL, 2);
	sizerTime->Add(timeTrack, 1, wxALIGN_CENTRE | wxEXPAND | wxALL, 2);

	optionbar = new wxToolBar(this, ID_TOOLBAR, wxPoint(0, 0),
			wxSize(prefTreeViewWidth, toolBarHeight), wxTB_HORIZONTAL);
	// wxBitmap optionBarBitmaps[2];
	optionbar->SetToolBitmapSize(wxSize(16, 16));
	optionbar->AddTool(Tool_linkorder, _T("Link Order"), toolBarBitmaps[3],
			toolBarBitmaps[3], wxITEM_CHECK, _T("Fix link position"));
	optionbar->AddTool(Tool_checkcollisions, _T("Check Collisions"),
			toolBarBitmaps[4], toolBarBitmaps[4], wxITEM_CHECK,
			_T("Click to toggle collision detection"));

	timeText = new wxTextCtrl(optionbar, 1008, wxT(" 0.00"), wxDefaultPosition,
			wxSize(50, 20), wxTE_PROCESS_ENTER | wxTE_RIGHT);
	optionbar->AddSeparator();
	optionbar->AddControl(timeText);

	CreateStatusBar(2);
	SetStatusText(wxT("GRIP Loading..."));

	/// Create sizers - these will manage the layout/resizing of the frame elements
	wxSizer *sizerFull = new wxBoxSizer(wxVERTICAL);
	wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);
	wxSizer *sizerRight = new wxBoxSizer(wxVERTICAL);
	wxSizer *sizerRightH = new wxBoxSizer(wxHORIZONTAL);
	wxSizer *sizerBottom = new wxBoxSizer(wxHORIZONTAL);

	treeView = new TreeView(
			this,
			TreeViewHandle,
			wxPoint(0, 0),
			wxSize(prefTreeViewWidth, prefViewerHeight - 2 * toolBarHeight),
			wxTR_LINES_AT_ROOT | wxTR_HAS_BUTTONS | wxTR_HIDE_ROOT
					| wxSUNKEN_BORDER);

	/// Adding a backPanel to the lower half of the window covers the generic "inner grey"
	/// with a forms-like control color. The tabView is added to the backPanel
	backPanel = new wxPanel(this, -1, wxDefaultPosition, wxDefaultSize, 0);
	tabView = new wxNotebook(backPanel, wxID_ANY, wxPoint(0, 0),
			wxSize(prefTreeViewWidth + prefViewerWidth, prefTabsHeight),
			wxNB_NOPAGETHEME | wxNB_TOP);

	addAllTabs();

	/// Start OpenGL and do various hacks to make it work cross platform
#ifndef WIN32 // Weird hack to make wxWidgets work in Linux
	Show();
#endif
	viewer = new Viewer(this, -1, wxPoint(0, 0),
			wxSize(prefViewerWidth, prefViewerHeight),
			wxFULL_REPAINT_ON_RESIZE | wxSUNKEN_BORDER);

#ifdef WIN32  // Weird hack to make wxWidgets work with VC++ debug
	viewer->MSWSetOldWndProc((WXFARPROC)DefWindowProc);
#endif

	/// Add elements to the sizers, setting the proportion flag to 1 and using the
	/// wxEXPAND flag to ensure that the Viewer fills the entire sizer (subject to constraints)
	sizerFull->Add(sizerTop, 1, wxEXPAND | wxALL, 0);
	sizerTop->Add(viewer, 1, wxALIGN_LEFT | wxEXPAND | wxALL, 0);

	/// Set the proportion flag to 0 (for wxHORIZONTAL sizer) to fix the width to its minimal size
	sizerTop->Add(sizerRight, 0, wxALIGN_RIGHT | wxEXPAND | wxALL, 0);

	sizerRight->Add(filebar, 0, wxALL, 0);
	sizerRight->Add(sizerRightH, 1, wxEXPAND | wxALL, 0);
	sizerRight->Add(optionbar, 0, wxALIGN_LEFT | wxEXPAND | wxALL, 0);

	sizerRightH->Add((wxTreeCtrl*) treeView, 1,
			wxALIGN_CENTRE | wxALL | wxEXPAND, 0);
	sizerRightH->Add(timePanel, 0, wxALIGN_LEFT | wxEXPAND | wxALL, 0);

	/// Place the back panel on the lower part of the window (0 fixes the height for wxVERTICAL sizer)
	sizerFull->Add(backPanel, 0, wxEXPAND | wxALL, 0);

	/// Add another sizer to stretch the tabs over the back panel while keeping a 3 pixel border
	sizerBottom->Add(tabView, 1, wxALIGN_LEFT | wxEXPAND | wxALL, 3);
	backPanel->SetSizer(sizerBottom);
	timePanel->SetSizer(sizerTime);

	SetSizer(sizerFull);
	sizerFull->SetSizeHints(this);
	filebar->Realize();
	optionbar->Realize();

	Show();
	viewer->Freeze();
	viewer->InitGL();
	viewer->ResetGL();
	viewer->Thaw();
}

/**
 * @function OnSaveScene
 * @brief Checks if the input file is a .rscene file or not and then writes the scene to it.
 */
void GRIPFrame::OnSaveScene(wxCommandEvent& WXUNUSED(event) ) {

	wxString filepath;
	string filename;
	size_t endpath;
	wxFileDialog *SaveDialog = new wxFileDialog(this, _("Save File As"),
			wxT("../scene/"), wxT(""), _("*.rscene"),
			wxFD_SAVE | wxFD_OVERWRITE_PROMPT, wxDefaultPosition);

	if (SaveDialog->ShowModal() == wxID_OK) {

		mCamRadius = viewer->camRadius;
		mCamRotT = viewer->camRotT;
		mWorldV = viewer->worldV;
		mBackColor = viewer->backColor;
		mGridColor = viewer->gridColor;

		filepath = SaveDialog->GetPath();
		filename = string(filepath.mb_str());
		endpath = filename.find(".rscene");

		if (endpath == (unsigned int) -1)
			filename += ".rscene";

		saveRscene(filename);
		wxString filename_string(filename.c_str(), wxConvUTF8);
		saveText(filename_string, ".lastload");
	}
}

/**
 * @function OnSaveRobot
 */
void GRIPFrame::OnSaveRobot(wxCommandEvent& WXUNUSED(event)) {

}

/**
 * @function OnLoad
 */
void GRIPFrame::OnLoad(wxCommandEvent& WXUNUSED(event)) {
	viewer->loading = true;
	wxString filename = wxFileSelector(wxT("Choose a file to open"),
			wxT("../scene/"), wxT(""), wxT(""), // -- default extension
			wxT("*.rscene"), 0);
	if (!filename.empty()) {
		DoLoad(string(filename.mb_str()));
	}
}

void GRIPFrame::OnPalletLoad(wxCommandEvent& WXUNUSED(event)) {
	SetStatusText(wxString("Select folder to load pallet data", wxConvUTF8));

	const wxString& dir = wxDirSelector(wxT("Choose a folder"));
	if (!dir.empty()) {
		i.load(dir.To8BitData());
	}

	// i.print();
	d.get_input(i);
	i.load(dir.To8BitData());

	if (!d.importdb()) {
		SetStatusText(wxString("Failed to import database", wxConvUTF8));
	}
	SetStatusText(wxString("Pallet data imported", wxConvUTF8));

	// d.printdb();

	treeView->CreateFromDatabase();
	updateAllTabs();
}

/**
 * @function OnQuickLoad
 * @brief Load the latest correctly loaded .rscene
 */
void GRIPFrame::OnQuickLoad(wxCommandEvent& WXUNUSED(event)) {

	viewer->loading = true;
	ifstream lastloadFile;
	lastloadFile.open(".lastload", ios::in);
	if (lastloadFile.fail()) {
		cout << "--(!) No previously loaded files (!)--" << endl;
		return;
	}
	string line;
	getline(lastloadFile, line);
	lastloadFile.close();
	DoLoad(line);
}

/**
 * @function DoLoad
 * @brief Load world from RSDH file
 */
void GRIPFrame::DoLoad(string filename) {

	DeleteWorld();

	mWorld = parseWorld(string(filename));
	mWorld->printInfo();
	// Create Collision
	mCollision = new Collision();
	mCollision->InitFromWorld(mWorld);

	// UpdateTreeView();
	cout << "--(v) Done Parsing World information (v)--" << endl;
	treeView->CreateFromWorld();
	cout << "--(v) Done Updating TreeView (v)--" << endl;
	// viewer->ResetGL();
	SetStatusText(wxT("--(i) Done Loading and updating the View (i)--"));

	/// Extract path to executable & save "lastload" there
	cout << "--(i) Saving " << filename << " to .lastload file (i)--" << endl;
	wxString filename_string(filename.c_str(), wxConvUTF8);
	saveText(filename_string, ".lastload");

	selectedTreeNode = 0;
	treeView->ExpandAll();
	updateAllTabs();

	viewer->ResetGL();
}

/**
 * @function DeleteWorld
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::DeleteWorld() {

	InitTimer("", 0);

	for (size_t i = 0; i < timeVector.size(); i++) {
		if (timeVector[i] != NULL)
			delete timeVector[i];
	}
	timeVector.clear();

	if (mWorld != NULL) {
		planning::World* w = mWorld;
		mWorld = 0;
		selectedTreeNode = 0;
		treeView->DeleteAllItems();
		delete w;
	}

}

/**
 * @function OnToolOrder
 */
void GRIPFrame::OnToolOrder(wxCommandEvent& WXUNUSED(event)) {
	reverseLinkOrder = !reverseLinkOrder;
}

/**
 * @function OnToolCheckColl
 * @brief 
 */
void GRIPFrame::OnToolCheckColl(wxCommandEvent& WXUNUSED(event)) {
	/// Toggle collision detection
	check_for_collisions = !check_for_collisions;
}

/**
 * @function OnToolMovie
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::OnToolMovie(wxCommandEvent& event) {
	wxString dirname = wxDirSelector(
			wxT("Choose output directory for movie pictures:")); // , "", wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST

	if (dirname.empty()) { // filename
		std::cout << "No Directory Selected" << std::endl;
		return;
	}

	string path = string(dirname.mb_str());

	char *buf = new char[1000];
	int w, h;

	double step = 1.0;//.03333/tIncrement;
	int count = 0;

	wxClientDC dc2(viewer);
	dc2.GetSize(&w, &h);

	for (double s = 0; s < timeVector.size(); s += step) {
		int i = (int) s;

		timeVector[i]->SetToWorld(mWorld);
		viewer->UpdateCamera();
		wxYield();

		unsigned char* imageData = (unsigned char*) malloc(w * h * 3);
		glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, imageData);
		wxImage img_ud(w, h, imageData);
		wxImage img = img_ud.Mirror(false);

		sprintf(buf, "%s/%06d.png", path.c_str(), count);

		wxString fname = wxString(buf, wxConvUTF8);
		cout << "Saving:" << buf << ":" << endl;
		img.SaveFile(fname, wxBITMAP_TYPE_PNG);

		count++;
	}

	delete buf;
	event.Skip();
}

/**
 * @function OnToolScreenshot
 * @brief Save a screenshot of the current GUI (Viewer and whole frame)
 * @date 2011-10-13
 */
void GRIPFrame::OnToolScreenshot(wxCommandEvent& WXUNUSED(event)) {
	wxYield();

	int w, h;
	wxClientDC dc(viewer);
	dc.GetSize(&w, &h);

	unsigned char* imageData = (unsigned char*) malloc(w * h * 3);
	glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, imageData);
	wxImage img_ud(w, h, imageData);
	wxImage img = img_ud.Mirror(false);
	img.SaveFile(wxT("ScreenGL.png"), wxBITMAP_TYPE_PNG); // Save GL image
	wxBitmap glbit(img); // Create a bitmap from GL image

	int w2, h2;
	wxClientDC dc2(this);
	dc2.GetSize(&w2, &h2);
	wxBitmap bit;
	bit.Create(w2, h2);
	wxMemoryDC mem(bit);
	mem.Blit(0, 0, w2, h2, &dc2, 0, 0); // Blit the window
	mem.DrawBitmap(glbit, 2, 2); // Draw the GL image
	bit.SaveFile(wxT("Screen.png"), wxBITMAP_TYPE_PNG);

	SetStatusText(wxT("Screenshots saved in: Screen.png and ScreenGL.png"));
}

/**
 * @function saveText
 * @brief
 * @date 2011-10-13
 */int GRIPFrame::saveText(wxString scenepath, const char* llfile) {
	try {
		ofstream lastloadFile(llfile, ios::out);
		if (lastloadFile) {
			lastloadFile << string(scenepath.mb_str()) << endl;
			lastloadFile.close();
			cout << "Saved" << endl;
			return 0;
		} else {
			cout << "Error opening file: " << llfile << endl;
			return -1;
		}
	} catch (const std::exception& e) {
		cout
				<< "Shouldn't see this catch after migrating to wxWidgets... tell jon: "
				<< e.what() << endl;
		return 0;
	}
	return 1;
}

/**
 * @function OnClose
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::OnClose(wxCommandEvent& WXUNUSED(event)) {
	DeleteWorld();
	//viewer->UpdateCamera();
	//exit(0);
	//DeleteWorld();
}

/**
 * @function OnQuit
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::OnQuit(wxCommandEvent& WXUNUSED(event)) {
	exit(0);
	//Close(true);
}

/**
 * @function OnAbout
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::OnAbout(wxCommandEvent& WXUNUSED(event)) {
	wxMessageBox(
			wxString::Format(
					wxT("GRIP: GT Humanoid Robotics Lab \
                                      \n\n Mike Stilman, Saul Reynolds-Haertle, Jonathan Scholz\
									  \n Pushkar Kolhe, Tobias Kunz, Ana Huaman"),
					wxVERSION_STRING, wxGetOsDescription().c_str()),
			wxT("Info"), wxOK | wxICON_INFORMATION, this);
}

/**
 * @function OnTvChange
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::onTVChange(wxTreeEvent& WXUNUSED(event)) {
	updateAllTabs();
}

/**
 * @function updateAllTabs
 * @brief Go through all the tabs and indicate a state change
 * @date 2011-10-13
 */
void GRIPFrame::updateAllTabs() {

	int type = 0;
	wxCommandEvent evt(wxEVT_GRIP_STATE_CHANGE, GetId());
	evt.SetEventObject(this);
	evt.SetClientData((void*) &type);
	size_t numPages = tabView->GetPageCount();
	for (size_t i = 0; i < numPages; i++) {
		GRIPTab* tab = (GRIPTab*) tabView->GetPage(i);
		tab->GRIPStateChange();
	}
}

/**
 * @function setTimeValue
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::setTimeValue(double value, bool sendSignal) {
	tCurrent = value;
	timeTrack->SetValue(value * tPrecision);
	updateTimeValue(value, sendSignal);
}

/**
 * @function updateTimeValue
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::updateTimeValue(double value, bool sendSignal) {
	value = 0.0f;
	if (tIncrement == 0)
		return;
	char buf[100];
	sprintf(buf, "%6.2f", tCurrent);

	wxString posString = wxString(buf, wxConvUTF8);
	timeText->ChangeValue(posString);

	unsigned int timeIndex = (int) ((tCurrent / tMax)
			* ((double) timeVector.size()));
	if (timeIndex > timeVector.size() - 1)
		timeIndex = timeVector.size() - 1;
	timeVector[timeIndex]->SetToWorld(mWorld);
	viewer->UpdateCamera();

	if (sendSignal)
		updateAllTabs();
}

/**
 * @function OnTimeScroll
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::OnTimeScroll(wxScrollEvent& event) {
	tCurrent = (double) (event.GetPosition()) / (double) tPrecision;
	//updateTimeValue(tCurrent);
	updateTimeValue(tCurrent, true);
}

/**
 * @function AddWorld
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::AddWorld(planning::World* _world) {
	GRIPTimeSlice* tsnew = new GRIPTimeSlice(_world);
	timeVector.push_back(tsnew);
	tMax += tIncrement;
	timeTrack->SetRange(0, tMax * tPrecision);
}

/**
 * @function InitTimer
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::InitTimer(string /* title */, double period) {

	for (size_t i = 0; i < timeVector.size(); i++) {
		delete timeVector[i];
	}
	tMax = 0;
	timeVector.clear();
	tIncrement = period;
}

/**
 * @function OnTimeEnter 
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::OnTimeEnter(wxCommandEvent& WXUNUSED(event)) {
	double p;
	timeText->GetValue().ToDouble(&p);
	if (p < 0)
		p = 0;
	if (p > tMax)
		p = tMax;
	setTimeValue(p);
	setTimeValue(p, true);
}

/**
 * @function OnWhite
 * @brief Set scene background to white 
 * @date 2011-10-13
 */
void GRIPFrame::OnWhite(wxCommandEvent& WXUNUSED(event)) {
	viewer->backColor = Vector3d(1, 1, 1);
	viewer->gridColor = Vector3d(.8, .8, 1);
	viewer->setClearColor();
	viewer->UpdateCamera();
}

/**
 * @function OnBlack
 * @brief Set scene background to black
 * @date 2011-10-13
 */
void GRIPFrame::OnBlack(wxCommandEvent& WXUNUSED(event)) {

	viewer->backColor = Vector3d(0, 0, 0);
	viewer->gridColor = Vector3d(.5, .5, 0);
	viewer->setClearColor();
	viewer->UpdateCamera();
}

/**
 * @function OnCameraReset 
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::OnCameraReset(wxCommandEvent& WXUNUSED(event)) {
	viewer->ResetGL();
}
BEGIN_EVENT_TABLE(GRIPFrame, wxFrame)
				EVT_COMMAND_SCROLL(1009, GRIPFrame::OnTimeScroll) EVT_TEXT_ENTER(1008, GRIPFrame::OnTimeEnter)

				EVT_MENU(MenuSaveScene, GRIPFrame::OnSaveScene)
				EVT_MENU(MenuSaveRobot, GRIPFrame::OnSaveRobot)
				EVT_MENU(MenuLoad, GRIPFrame::OnLoad)
				EVT_MENU(MenuQuickLoad, GRIPFrame::OnQuickLoad)
				EVT_MENU(wxID_CLOSE, GRIPFrame::OnClose)
				EVT_MENU(MenuClose, GRIPFrame::OnClose)
				EVT_MENU(MenuQuit, GRIPFrame::OnQuit)
				EVT_MENU(MenuAbout, GRIPFrame::OnAbout)

				EVT_MENU(MenuBgWhite, GRIPFrame::OnWhite)
				EVT_MENU(MenuBgBlack, GRIPFrame::OnBlack)
				EVT_MENU(MenuCameraReset, GRIPFrame::OnCameraReset)

				EVT_MENU(wxID_OPEN, GRIPFrame::OnLoad)
				EVT_MENU(Tool_quickload, GRIPFrame::OnQuickLoad)
				EVT_MENU(wxID_SAVE, GRIPFrame::OnSaveScene)

				EVT_MENU(Tool_linkorder, GRIPFrame::OnToolOrder)
				EVT_MENU(Tool_checkcollisions, GRIPFrame::OnToolCheckColl)
				EVT_MENU(Tool_screenshot, GRIPFrame::OnToolScreenshot)
				EVT_MENU(Tool_movie, GRIPFrame::OnToolMovie)

				EVT_MENU(MenuPalletLoad, GRIPFrame::OnPalletLoad)

				EVT_TREE_SEL_CHANGED(TreeViewHandle,GRIPFrame::onTVChange)

				//	EVT_BUTTON (BUTTON_Hello, GRIPFrame::OnQuit )
				END_EVENT_TABLE()

