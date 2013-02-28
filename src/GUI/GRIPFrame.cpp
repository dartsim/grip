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
#include <limits>
#include <ctime>

#include "GUI.h"
#include "GRIPFrame.h"
#include "Viewer.h"
#include "TreeView.h"

#include <Tabs/AllTabs.h>
#include <Tabs/GRIPTab.h>
#include "GRIPSlider.h"
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
#include "icons/simulate.xpm"
#include "icons/play.xpm"
#include "icons/stop.xpm"
#include "icons/rightSideView.xpm"
#include "icons/frontView.xpm"
#include "icons/topView.xpm"

#include <robotics/Robot.h>
#include <kinematics/ShapeBox.h> // for floor
#include <kinematics/Joint.h> // for floor
#include <kinematics/TrfmTranslate.h> // for floor
#include <kinematics/TrfmRotateEuler.h> // for floor
#include <kinematics/Dof.h>

// Parser
#include <robotics/parser/dart_parser/DartLoader.h>

#define ID_TOOLBAR 1257
#define ID_TIMESLIDER 1258

enum toolNums{
    Tool_open= 1262,
    Tool_save= 1263,
    Tool_quickload = 1264,
    
    Tool_linkorder = 1265,
    Tool_checkcollisions = 1266,
    Tool_screenshot = 1267,
    Tool_movie = 1268,

    Tool_rightSideView = 1269,   ///< Change the view to one of the orthogonal options
    Tool_frontView = 1270,
    Tool_topView = 1271
};

extern bool check_for_collisions;

//wxSTD_MDIPARENTFRAME ICON wxICON(ROBOT_xpm)

/**
 * @function GRIPFrame
 * @brief Constructor 
 * @date 2011-10-13
 */
GRIPFrame::GRIPFrame(const wxString& title) : wxFrame(NULL, wxID_ANY, title) {

    tPrecision = 1000;
    tMax = 5;
    tMax = 0;
    InitTimer("",0);
    std::cout << "GRIPFrame 1" << std::endl;

    continueSimulation = false;
    timeLastRedraw = -1;
    renderDuringSimulation = true;
    filteredRelSimSpeed = 8;

    // ========================================================
    // A. Create the menu bar
    
    wxMenu *fileMenu = new wxMenu;
    wxMenu *viewMenu = new wxMenu;
    wxMenu *simMenu = new wxMenu;
    wxMenu *settingsMenu = new wxMenu;
    wxMenu *renderMenu = new wxMenu;
    wxMenu *bgMenu = new wxMenu;
    wxMenu *helpMenu = new wxMenu;
    
    // Create the file menu
    fileMenu->Append(MenuLoad, wxT("L&oad\tAlt-O"));
    fileMenu->Append(MenuQuickLoad, wxT("Q&uickLoad\tAlt-Shift-Q"));
    fileMenu->Append(MenuSaveScene, wxT("Save Scene"));
    //saveMenu->Append(MenuSaveRobot, wxT("Save Robot"));
    //fileMenu->AppendSubMenu(saveMenu, wxT("S&ave\tAlt-S"));
    fileMenu->Append(MenuClose, wxT("C&lose\tAlt-C"));
    fileMenu->AppendSeparator();
    fileMenu->Append(MenuQuit, wxT("E&xit\tAlt-Q"));

    // Create the view menu - constrain the camera position/orientation
    viewMenu->Append(MenuFrontView, wxT("&Front"));
    viewMenu->Append(MenuTopView, wxT("&Top"));
    viewMenu->Append(MenuRightSideView, wxT("&Side"));

    // Create the simulation menu
    simMenu->Append(MenuSimulateStart, wxT("Start simulation"));
    simMenu->Append(MenuSimulateStop, wxT("Stop Simulation"));
    simMenu->Append(MenuSimulateSingle, wxT("Simulate Single Step"));

    // Create the background menu
    bgMenu->Append(MenuBgWhite, wxT("White"));
    bgMenu->Append(MenuBgBlack, wxT("Black"));

    // Create the render menu
    renderMenu->Append(MenuRenderXGA, wxT("XGA 1024x768"));
    renderMenu->Append(MenuRenderVGA, wxT("VGA 640x480"));
    renderMenu->Append(MenuRenderHD, wxT("HD 1280x720"));

    // Create the settings menu
    settingsMenu->AppendCheckItem(MenuRenderDuringSim, wxT("Render During Simulation"));
    settingsMenu->AppendSubMenu(bgMenu, wxT("Background"));
    settingsMenu->Append(MenuCameraReset, wxT("Reset Camera"));
    settingsMenu->Check(MenuRenderDuringSim, true);
    
    // Create the help menu
    helpMenu->Append(MenuAbout, wxT("&About...\tF1"), wxT("Show about dialog"));

    // Add all the menus to the menu bar
    wxMenuBar *menuBar = new wxMenuBar();
    menuBar->Append(fileMenu, wxT("&File"));
    menuBar->Append(viewMenu, wxT("&View"));
    menuBar->Append(simMenu, wxT("S&imulation"));
    menuBar->Append(settingsMenu, wxT("&Settings"));
    menuBar->Append(renderMenu, wxT("&Render"));
    menuBar->Append(helpMenu, wxT("&Help"));

    // ========================================================
    // B. Create the toolbar icons for shortcuts
    
    // Get the bitmaps
    toolBarBitmaps[0] = wxIcon(open_xpm);
    toolBarBitmaps[1] = wxIcon(save_xpm);
    toolBarBitmaps[2] = wxIcon(redo_xpm);
    toolBarBitmaps[3] = wxIcon(anchor_xpm);
    toolBarBitmaps[4] = wxIcon(asterisk_xpm);
    toolBarBitmaps[5] = wxIcon(camera_xpm);
    toolBarBitmaps[6] = wxIcon(film_xpm);
    toolBarBitmaps[7] = wxIcon(simulate_xpm);
    toolBarBitmaps[8] = wxIcon(play_xpm);
    toolBarBitmaps[9] = wxIcon(stop_xpm);
    toolBarBitmaps[10] = wxIcon(topView_xpm);
    toolBarBitmaps[11] = wxIcon(rightSideView_xpm);
    toolBarBitmaps[12] = wxIcon(frontView_xpm);

    wxBitmap clockBmp = wxBitmap(clock_xpm);

    // Create the toolbar and assign the callback functions
    filebar = new wxToolBar(this,ID_TOOLBAR,wxPoint(0, 0), wxSize(prefViewerWidth+prefTreeViewWidth+50, toolBarHeight), wxTB_HORIZONTAL);
    filebar->SetToolBitmapSize(wxSize(16, 16));
    filebar->AddTool(wxID_OPEN, _T("Open"),toolBarBitmaps[0], toolBarBitmaps[0], wxITEM_NORMAL, _T("Open .rscene file (Alt-O)"));
    // filebar->AddTool(wxID_SAVE, _T("Save"),toolBarBitmaps[1], toolBarBitmaps[1], wxITEM_NORMAL,  _T("Save world to .rscene file (Alt-S)"));
    // filebar->AddSeparator();
    filebar->AddTool(Tool_quickload, _T("Quick Load"),toolBarBitmaps[2], toolBarBitmaps[2], wxITEM_NORMAL, _T("Load last viewed scene (Alt-Shift-Q)"));
    filebar->AddSeparator();
    filebar->AddTool(MenuSimulateStart, _T("Start Simulation"),toolBarBitmaps[7], toolBarBitmaps[7], wxITEM_NORMAL, _T("Start Simulation"));
    // filebar->AddTool(MenuPlay, _T("Play"),toolBarBitmaps[8], toolBarBitmaps[8], wxITEM_NORMAL, _T("Play"));
    filebar->AddTool(MenuSimulateStop, _T("Stop Simulation"),toolBarBitmaps[9], toolBarBitmaps[9], wxITEM_NORMAL, _T("Stop Simulation"));
    filebar->AddSeparator();
    filebar->AddTool(Tool_screenshot, _T("Screenshot"),toolBarBitmaps[5], toolBarBitmaps[5], wxITEM_NORMAL, _T("Export screenshot"));
    filebar->AddTool(Tool_movie, _T("Movie"),toolBarBitmaps[6], toolBarBitmaps[6], wxITEM_NORMAL, _T("Export film sequence"));
    
    // Add the optional views such as top, front and side
    filebar->AddSeparator();
    filebar->AddTool(Tool_frontView, _T("frontView"),toolBarBitmaps[12], toolBarBitmaps[12], wxITEM_NORMAL, _T("View scene from front"));
    filebar->AddTool(Tool_topView, _T("topView"),toolBarBitmaps[10], toolBarBitmaps[10], wxITEM_NORMAL, _T("View scene from top"));
    filebar->AddTool(Tool_rightSideView, _T("rightSideView"),toolBarBitmaps[11], toolBarBitmaps[11], wxITEM_NORMAL, _T("View scene from right"));

    // ========================================================
    // C. Create the time slider
    
    //timeSlider = new GRIPSlider(clockBmp,0,1000,100,0,100,500,this,ID_TIMESLIDER,true);
    wxPanel* timePanel = new wxPanel(this, -1, wxDefaultPosition, wxDefaultSize, 0);
    
#ifdef WIN32 /// For windows use a thicker slider - it looks nice
    timeTrack = new wxSlider(timePanel,1009,0,0,1000,wxDefaultPosition, wxSize(30,100), wxSL_BOTH | wxSL_VERTICAL | wxALIGN_CENTRE);
#else
    timeTrack = new wxSlider(timePanel,1009,0,0,1000,wxDefaultPosition, wxSize(16,100), wxSL_BOTH | wxSL_VERTICAL | wxALIGN_CENTRE);
#endif
    wxStaticBitmap *timeButton = new wxStaticBitmap(timePanel, -1, clockBmp, wxDefaultPosition, wxSize(16,16), wxALIGN_CENTRE);
    wxSizer *sizerTime = new wxBoxSizer(wxVERTICAL);
    sizerTime->Add(timeButton,0 , wxALIGN_CENTRE | wxEXPAND | wxALL, 2);
    sizerTime->Add(timeTrack,1 , wxALIGN_CENTRE | wxEXPAND | wxALL, 2);

    // ========================================================
    // D. Create the option bar
    
    optionbar = new wxToolBar(this,ID_TOOLBAR,wxPoint(0, 0), wxSize(prefTreeViewWidth+50, toolBarHeight), wxTB_HORIZONTAL);
    // wxBitmap optionBarBitmaps[2];
    optionbar->SetToolBitmapSize(wxSize(16, 16));
    optionbar->AddTool(Tool_linkorder, _T("Link Order"),toolBarBitmaps[3], toolBarBitmaps[3], wxITEM_CHECK, _T("Fix link position"));
    optionbar->AddTool(Tool_checkcollisions, _T("Check Collisions"),toolBarBitmaps[4], toolBarBitmaps[4], wxITEM_CHECK, _T("Click to toggle collision detection"));

    timeText = new wxTextCtrl(optionbar,1008,wxT(" 0.00"),wxDefaultPosition,wxSize(50,20),wxTE_PROCESS_ENTER | wxTE_RIGHT);
    optionbar->AddSeparator();
    optionbar->AddControl(timeText);

    timeRelText = new wxTextCtrl(optionbar,1008,wxT("1.00"),wxDefaultPosition,wxSize(50,20),wxTE_PROCESS_ENTER | wxTE_RIGHT);
    optionbar->AddControl(timeRelText);

    // ========================================================
    // E. Create the status bar
    
    CreateStatusBar(2);
    SetStatusText(wxT("GRIP Loading..."));
    
    // ========================================================
    // F. Create the layout: treeView, tabView and 3D viewer.
    
    // Create sizers - these will manage the layout/resizing of the frame elements
    wxSizer *sizerFull = new wxBoxSizer(wxVERTICAL);
    wxSizer *sizerTop = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *sizerRight = new wxBoxSizer(wxVERTICAL);
    wxSizer *sizerRightH = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *sizerBottom = new wxBoxSizer(wxHORIZONTAL);
    
    // ********************************************
    // F1. Add the filebar 

    sizerFull->Add(filebar,0, wxALIGN_TOP | wxALL, 0);

    // ********************************************
    // F2a. Create the LHS of the top sizer: 3D
    
    // Create the 3D viewer
#ifndef WIN32 // Weird hack to make wxWidgets work in Linux
    Show();
#endif
    {
      int attrib[] = {
	WX_GL_DOUBLEBUFFER,
	WX_GL_RGBA,
	WX_GL_DEPTH_SIZE, 16,
	0
      };
      viewer = new Viewer(this, NULL, -1, wxPoint(0, 0), wxSize(prefViewerWidth, prefViewerHeight),
			  wxFULL_REPAINT_ON_RESIZE | wxSUNKEN_BORDER, _T("GLCanvas"), attrib);
    }
#ifdef WIN32  // Weird hack to make wxWidgets work with VC++ debug
    viewer->MSWSetOldWndProc((WXFARPROC)DefWindowProc);
#endif
    
    // Add the viewer to the sizer
    sizerTop->Add(viewer, 1, wxALIGN_LEFT | wxEXPAND | wxALL, 0);
    
    // ********************************************
    // F2b. Create the RHS of the top sizer
    
    // Create the treeview that will go to the right handside of the top sizer. kHackOffset is referenced below.
    static const size_t kHackOffset = 30;
    treeView = new TreeView(this, TreeViewHandle, wxPoint(0, 0), wxSize(prefTreeViewWidth, prefViewerHeight-2*toolBarHeight - kHackOffset),
                            wxTR_LINES_AT_ROOT | wxTR_HAS_BUTTONS | wxTR_HIDE_ROOT | wxSUNKEN_BORDER);
    sizerRightH->Add((wxTreeCtrl*)treeView, 1, wxALIGN_CENTRE | wxALL | wxEXPAND, 0);

    // Also add the time panel created above. Note that RightH is horizontal
    // so we need to add treeView and timePanel next to each other.
    timePanel->SetSizer(sizerTime);
    sizerRightH->Add(timePanel,0, wxALIGN_LEFT | wxEXPAND | wxALL, 0);
    
    // Add the three parts of the right handside vertically. sizerRight is vertical.
    // sizerRight->Add(filebar,0, wxALIGN_TOP | wxALL, 0);
    sizerRight->Add(sizerRightH, 1 , wxALIGN_TOP | wxEXPAND | wxALL, 0);
    sizerRight->Add(optionbar,0, wxALIGN_TOP | wxALL, 0);
    
    // HACK: To make sure that the optionbar is not blocked by the sizerRHS
    wxSize rs = sizerRight->GetMinSize();
    sizerRight->SetMinSize(rs.x, rs.y + kHackOffset);
    
    // Finally, add the RHS to the top sizer:
    /// Set the proportion flag to 0 (for wxHORIZONTAL sizer) to fix the width to its minimal size
    sizerTop->Add(sizerRight, 0, wxALIGN_RIGHT | wxEXPAND | wxALL, 0);
    
    // End of F1: Add the top to the full sizer:
    // Add elements to the sizers, setting the proportion flag to 1 and using the
    // wxEXPAND flag to ensure that the Viewer fills the entire sizer (subject to constraints)
    sizerFull->Add(sizerTop, 1,  wxEXPAND | wxALL,  0 );
    
    // ********************************************
    // F3. Create the bottom sizer 
    
    /// Adding a backPanel to the lower half of the window covers the generic "inner grey"
    /// with a forms-like control color. The tabView is added to the backPanel
    backPanel = new wxPanel(this, -1, wxDefaultPosition, wxDefaultSize, 0);
    tabView = new wxNotebook(backPanel, wxID_ANY, wxPoint(0, 0), wxSize(prefTreeViewWidth+prefViewerWidth, prefTabsHeight),  wxNB_NOPAGETHEME | wxNB_TOP);
    addAllTabs();

    /// Add another sizer to stretch the tabs over the back panel while keeping a 3 pixel border
    sizerBottom->Add(tabView, 1, wxALIGN_LEFT | wxEXPAND | wxALL, 3);
    backPanel->SetSizer(sizerBottom);

    /// Place the back panel on the lower part of the window (0 fixes the height for wxVERTICAL sizer)
    sizerFull->Add(backPanel, 0, wxEXPAND | wxALL, 0);

    // ========================================================
    // G. Set the fullSizer as the frame's sizer and wrap up
    
    // Set the sizer
    SetSizer(sizerFull);
    sizerFull->SetSizeHints( this );
    filebar->Realize();
    optionbar->Realize();
    
    // Start the OpenGL visualization
    Show();
    viewer->Freeze();
    viewer->InitGL();
    viewer->DrawGLScene();
    viewer->Thaw();

    SetMenuBar(menuBar);
}

/**
 * @function OnSaveScene
 * @brief Checks if the input file is a .rscene file or not and then writes the scene to it.
 */
void GRIPFrame::OnSaveScene( wxCommandEvent& WXUNUSED(event) ) {

    wxString filepath;
    string filename;
    size_t endpath;
    wxFileDialog *SaveDialog = new wxFileDialog( this, _("Save File As"), wxT("../scene/"), wxT(""),
			                         _("*.rscene"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT, wxDefaultPosition );

    if (SaveDialog->ShowModal() == wxID_OK) {

	filepath = SaveDialog->GetPath();
	filename = string(filepath.mb_str());
	endpath = filename.find(".rscene");

	if(endpath == (unsigned int)-1) filename += ".rscene";

	saveRscene( filename );
	wxString filename_string(filename.c_str(), wxConvUTF8);
	saveText(filename_string,".lastload");
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
void GRIPFrame::OnLoad(wxCommandEvent& WXUNUSED(event)){
    wxString filename = wxFileSelector(wxT("Choose a file to open"),wxT("../scene/"),wxT(""),wxT(""), // -- default extension
                                       wxT("*.urdf"), 0);
    if ( !filename.empty() ) {
        DoLoad(string(filename.mb_str()));
    }
}

/**
 * @function OnQuickLoad
 * @brief Load the latest correctly loaded .rscene
 */
void GRIPFrame::OnQuickLoad(wxCommandEvent& WXUNUSED(event)) {

    ifstream lastloadFile;
    lastloadFile.open(".lastload", ios::in);
    if(lastloadFile.fail()){
        cout << "--(!) No previously loaded files (!)--" << endl;
	return;
    }
    string line;
    getline(lastloadFile,line);
    lastloadFile.close();
    DoLoad(line);
}

/**
 * @function DoLoad
 * @brief Load world from RSDH file
 */
void GRIPFrame::DoLoad(string filename)
{
    continueSimulation = false;

    size_t numPages = tabView->GetPageCount();
    if (mWorld) {
        // fire SceneUnloaded hooks
        for(size_t i=0; i< numPages; i++) {
            GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
            tab->GRIPEventSceneUnloaded();
        }
        // delete the world
        DeleteWorld();
    }

    DartLoader dl;

    mWorld = dl.parseWorld( filename.c_str() );
    // NULL World
    if( !mWorld ) {
      std::cout<< "[GRIP] World pointer null. Try again with a good URDF file"<<std::endl;
      return;
    }
    else {
      // Empty world
      if( mWorld->getNumRobots() == 0 && mWorld->getNumObjects() == 0 ) {
	std::cout<< "[GRIP] Empty world? Neither robots nor objects loaded. Try again loading a world urdf"<< std::endl;
	return;
      }
      else {
	// A world with at least an object or a robot
	mWorld->printInfo();
	
	// Add floor
	robotics::Robot* ground = new robotics::Robot();
	ground->setName("ground");

	kinematics::Joint* joint;
	dynamics::BodyNodeDynamics* node;
	kinematics::Transformation* trans;	

	// Set the initial Rootnode that controls the position and orientation of the whole robot
	node = (dynamics::BodyNodeDynamics*) ground->createBodyNode("rootBodyNode");
	node->setShape(new kinematics::Shape());
	joint = new kinematics::Joint( NULL, node, "rootJoint" );
	
	// Add DOFs for RPY and XYZ of the whole robot
	trans = new kinematics::TrfmTranslateX( new kinematics::Dof( 0, "rootX" ), "Tx" );
	joint->addTransform( trans, true );
	ground->addTransform( trans );
	
	trans = new kinematics::TrfmTranslateY( new kinematics::Dof( 0, "rootY" ), "Ty" );
	joint->addTransform( trans, true );
	ground->addTransform( trans );
	
	trans = new kinematics::TrfmTranslateZ( new kinematics::Dof( 0, "rootZ" ), "Tz" );
	joint->addTransform( trans, true );
	ground->addTransform( trans );
	
	trans = new kinematics::TrfmRotateEulerZ( new kinematics::Dof( 0, "rootYaw" ), "Try" );
	joint->addTransform( trans, true );
	ground->addTransform( trans );
	
	trans = new kinematics::TrfmRotateEulerY( new kinematics::Dof( 0, "rootPitch" ), "Trp" );
	joint->addTransform( trans, true );
	ground->addTransform( trans );
	
	trans = new kinematics::TrfmRotateEulerX( new kinematics::Dof( 0, "rootRoll" ), "Trr" );
	joint->addTransform( trans, true );
	ground->addTransform( trans );
	
	// Set this first node as root node
	ground->addNode( node );
	ground->initSkel();    

	node = new dynamics::BodyNodeDynamics();
	node->setShape(new kinematics::ShapeBox(Eigen::Vector3d(10.0, 10.0, 0.0001), 1.0));
	joint = new kinematics::Joint(ground->getRoot(), node);
	ground->addNode(node);
	ground->initSkel();
	ground->update();
	ground->setImmobileState(true);
	mWorld->addObject(ground);
	mWorld->rebuildCollision();
	
	// Compile OpenGL displaylists
	for(int i=0; i < mWorld->getNumSkeletons(); i++) {
	  viewer->renderer.compileList(mWorld->getSkeleton(i));
	}
	
	
	// UpdateTreeView();
	cout << "--(v) Done Parsing World information (v)--" << endl;
	treeView->CreateFromWorld();
	cout << "--(v) Done Updating TreeView (v)--" << endl;
	SetStatusText(wxT("--(i) Done Loading and updating the View (i)--"));
	
	/// Extract path to executable & save "lastload" there
	cout << "--(i) Saving " << filename << " to .lastload file (i)--" << endl;
	wxString filename_string(filename.c_str(), wxConvUTF8);
	saveText(filename_string,".lastload");
	
	selectedTreeNode = 0;
	treeView->ExpandAll();
	updateAllTabs();
	
	// fire SceneLoaded hooks
	for(size_t i=0; i< numPages; i++) {
	  GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	  tab->GRIPEventSceneLoaded();
	}
	
      } // end of else (world empty)
 
    } // end of else (world no NULL)

    viewer->DrawGLScene();
}

/**
 * @function DeleteWorld
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::DeleteWorld() {

    InitTimer("",0);

    timeVector.clear();

    if( mWorld != NULL) {
        robotics::World* w = mWorld;
	mWorld = 0;
	selectedTreeNode = 0;
	treeView->DeleteAllItems();
	delete w;
    }

}

/**
 * @function OnToolOrder
 */
void GRIPFrame::OnToolOrder(wxCommandEvent& WXUNUSED(event)){
	reverseLinkOrder = !reverseLinkOrder;
}

/**
 * @function OnToolCheckColl
 * @brief 
 */
void GRIPFrame::OnToolCheckColl(wxCommandEvent& WXUNUSED(event)){
	/// Toggle collision detection
	check_for_collisions = !check_for_collisions;
}

/**
 * @function OnToolMovie
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::OnToolMovie(wxCommandEvent& event){
    wxString dirname = wxDirSelector(wxT("Choose output directory for movie pictures:")); // , "", wxDD_DEFAULT_STYLE | wxDD_DIR_MUST_EXIST

    if ( dirname.empty() ){ // filename
        std::cout << "No Directory Selected" << std::endl;
	return;
    }

    string path = string(dirname.mb_str());

    char *buf = new char[1000];

	//Create a new Viewer Window
	wxFrame *movieFrame = new wxFrame(NULL,wxID_ANY, wxT("MovieWindow"),wxPoint(0, 0), wxSize(renderW,renderH),wxDEFAULT_FRAME_STYLE & ~ (wxRESIZE_BORDER | wxMAXIMIZE_BOX));
	//#ifndef WIN32 // Weird hack to make wxWidgets work in Linux
	movieFrame->Show();
	//#endif

	int attrib[] = {WX_GL_DOUBLEBUFFER,WX_GL_RGBA,	WX_GL_DEPTH_SIZE, 16,0};
	Viewer *movieViewer = new Viewer(movieFrame, viewer, wxID_ANY, wxPoint(0, 0), wxSize(renderW, renderH), 0, _T("MovieWindow"), attrib);
	
	//movieFrame.AddChild(
	#ifdef WIN32  // Weird hack to make wxWidgets work with VC++ debug
	movieViewer->MSWSetOldWndProc((WXFARPROC)DefWindowProc);
	#endif

	int w,h;

	movieViewer->Show(true);
    movieViewer->Freeze();
    wxClientDC dc2(movieViewer);
    dc2.GetSize(&w, &h);
    movieViewer->InitGL();
    movieViewer->Thaw();
	movieViewer->handleEvents = false;
	movieViewer->Show(true);

	movieViewer->camT = viewer->camT;
	movieViewer->camRotT = viewer->camRotT;
	movieViewer->camRadius = viewer->camRadius;
	movieViewer->worldV = viewer->worldV;

    double step = 1.0;//.03333/tIncrement;
    int count = 0;

    for( double s=0; s < timeVector.size(); s+= step) {
         int i = (int)s;
         mWorld->setState(timeVector[i].state);
		movieViewer->DrawGLScene();
		wxYield();

		unsigned char* imageData = (unsigned char*) malloc(w * h * 3);
		glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, imageData);
		wxImage img_ud(w,h,imageData);
		wxImage img = img_ud.Mirror(false);

		sprintf(buf, "%s/%06d.png",path.c_str(),count);

		wxString fname = wxString(buf,wxConvUTF8);
		cout << "Saving:" << buf << ":" << endl;
		img.SaveFile(fname, wxBITMAP_TYPE_PNG);

		count++;
    }

	delete movieViewer;
	delete movieFrame;
	viewer->InitGL();

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

    int w,h;
    wxClientDC dc(viewer);
    dc.GetSize(&w, &h);

    unsigned char* imageData = (unsigned char*) malloc(w * h * 3);
    glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, imageData);
    wxImage img_ud(w,h,imageData);
    wxImage img = img_ud.Mirror(false);
    img.SaveFile(wxT("ScreenGL.png"), wxBITMAP_TYPE_PNG ); // Save GL image
    wxBitmap glbit(img); // Create a bitmap from GL image

    int w2,h2;
    wxClientDC dc2(this);
    dc2.GetSize(&w2, &h2);
    wxBitmap bit;
    bit.Create(w2,h2);
    wxMemoryDC mem(bit);
    mem.Blit(0,0,w2,h2,&dc2,0,0); // Blit the window
    mem.DrawBitmap(glbit,2,2);    // Draw the GL image
    bit.SaveFile(wxT("Screen.png"), wxBITMAP_TYPE_PNG );

    SetStatusText(wxT("Screenshots saved in: Screen.png and ScreenGL.png"));
}

/**
 * @function saveText
 * @brief
 * @date 2011-10-13
 */
int GRIPFrame::saveText(wxString scenepath, const char* llfile) {
    try {
        ofstream lastloadFile(llfile, ios::out);
	if (lastloadFile) {
	    lastloadFile  << string(scenepath.mb_str()) << endl;
	    lastloadFile.close();
	    cout << "Saved" << endl;
	    return 0;
        } else {
	    cout << "Error opening file: " << llfile << endl;
	    return -1;
        }
    } catch (const std::exception& e) {
        cout << "Shouldn't see this catch after migrating to wxWidgets... tell jon: " << e.what() << endl;
	return 0;
    }
    return 1;
}

/**
 * @function OnClose
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::OnClose(wxCommandEvent& WXUNUSED(event)){
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
    wxMessageBox( wxString::Format( wxT("GRIP: GT Humanoid Robotics Lab \
                                      \n\n Mike Stilman, Saul Reynolds-Haertle, Jonathan Scholz\
									  \n Pushkar Kolhe, Tobias Kunz, Ana Huaman"),
                                    wxVERSION_STRING,
                                    wxGetOsDescription().c_str()
                                  ),wxT("Info"), wxOK | wxICON_INFORMATION,this);
}

/**
 * @function OnTvChange
 * @brief
 * @date 2011-10-13
 */
void GRIPFrame::onTVChange(wxTreeEvent& WXUNUSED(event)){
	updateAllTabs();
}

/**
 * @function updateAllTabs
 * @brief Go through all the tabs and indicate a state change
 * @date 2011-10-13
 */
void GRIPFrame::updateAllTabs() {
    size_t numPages = tabView->GetPageCount();
    for(size_t i=0; i< numPages; i++){
        GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	tab->GRIPStateChange();
    }
}


/**
 * @function setTimeValue
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::setTimeValue(double value, bool sendSignal){
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
    if(tIncrement == 0) return;
    char buf[100];
    sprintf(buf, "%6.2f", tCurrent);
    wxString posString = wxString(buf,wxConvUTF8);
    timeText->ChangeValue(posString);

    unsigned int timeIndex = (int)((tCurrent/tMax)*((double)timeVector.size()));
 
    if(timeIndex > timeVector.size()-1) timeIndex = timeVector.size()-1;
  
    mWorld->setState(timeVector[timeIndex].state);
    viewer->UpdateCamera();
    viewer->DrawGLScene();

    if(sendSignal) updateAllTabs();
}

/**
 * @function OnTimeScroll
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::OnTimeScroll(wxScrollEvent& event) {
    tCurrent = (double)(event.GetPosition())/(double)tPrecision;
   
    updateTimeValue(tCurrent,true); 
}

/**
 * @function AddWorld
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::AddWorld( robotics::World* _world) {
    GRIPTimeSlice tsnew;
    tsnew.time = _world->mTime;
    tsnew.state = _world->getState();
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
    tMax = 0;
    timeVector.clear();
    tIncrement = period;
}

/**
 * @function OnTimeEnter 
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::OnTimeEnter(wxCommandEvent& WXUNUSED(event)){
    double p;
    timeText->GetValue().ToDouble(&p);
    if(p < 0) p = 0;
    if(p > tMax) p = tMax;
    setTimeValue(p);
    setTimeValue(p,true);
}

/**
 * @function OnWhite
 * @brief Set scene background to white 
 * @date 2011-10-13
 */
void GRIPFrame::OnWhite(wxCommandEvent& WXUNUSED(event)){
    viewer->backColor = Vector3d(1,1,1);
    viewer->gridColor = Vector3d(.8,.8,1);
    viewer->setClearColor();
    viewer->DrawGLScene();
}

/**
 * @function OnBlack
 * @brief Set scene background to black
 * @date 2011-10-13
 */
void GRIPFrame::OnBlack(wxCommandEvent& WXUNUSED(event)) {

    viewer->backColor = Vector3d(0,0,0);
    viewer->gridColor = Vector3d(.5,.5,0);
    viewer->setClearColor();
    viewer->DrawGLScene();
}

/**
 * @function OnMenuRenderDuringSimulation
 * @brief Check item sets whether DART should render the world during simulations
 * @date 2012-01-18
 */
void GRIPFrame::OnMenuRenderDuringSimulation(wxCommandEvent& event){
    renderDuringSimulation = event.IsChecked();
}

/**
 * @function OnVGA
 * @brief Set rendering to XGA
 * @date 2011-10-13
 */
void GRIPFrame::OnVGA(wxCommandEvent& WXUNUSED(event)){
    renderW = vgaW;
	renderH = vgaH;
}


/**
 * @function OnXGA
 * @brief Set rendering to XGA
 * @date 2011-10-13
 */
void GRIPFrame::OnXGA(wxCommandEvent& WXUNUSED(event)){
    renderW = xgaW;
	renderH = xgaH;
}

/**
 * @function OnHD
 * @brief Set rendering to 720p HD
 * @date 2011-10-13
 */
void GRIPFrame::OnHD(wxCommandEvent& WXUNUSED(event)){
    renderW = hd720W;
	renderH = hd720H;
}

/**
 * @function OnCameraReset 
 * @brief 
 * @date 2011-10-13
 */
void GRIPFrame::OnCameraReset(wxCommandEvent& WXUNUSED(event)) {
  viewer->camRotT = AngleAxis<double>(DEG2RAD(-30.0), Vector3d(0.0, 1.0, 0.0));
  viewer->worldV = Vector3d(0.0, 0.0, 0.0);
  viewer->camRadius = 10.0;
  viewer->UpdateCamera();
  viewer->DrawGLScene();
}

/**
 * @function OnViewChange
 * @author Can Erdogan
 * @brief Updates the view to either front, side or top view
 * @date 2013-01-24
 */
void GRIPFrame::OnViewChange(wxCommandEvent& event) {
	if((event.GetId() == Tool_topView) || (event.GetId() == MenuTopView))
		viewer->camRotT = AngleAxis<double>(DEG2RAD(-90.0), Vector3d(0.0, 1.0, 0.0));
	else if((event.GetId() == Tool_rightSideView) || (event.GetId() == MenuRightSideView))
		viewer->camRotT = AngleAxis<double>(DEG2RAD(90.0), Vector3d(0.0, 0.0, 1.0));
	else if((event.GetId() == Tool_frontView) || (event.GetId() == MenuFrontView))
		viewer->camRotT = AngleAxis<double>(DEG2RAD(0.0), Vector3d(0.0, 1.0, 0.0));
	viewer->UpdateCamera();
	viewer->DrawGLScene();
}

/**
 * @function OnSimulateStart 
 * @brief Activate/Deactivate dynamic simulation
 * @date 2013-01-15
 */
void GRIPFrame::OnSimulateStart(wxCommandEvent& event) {
    if (!mWorld) {
        std::cout << "No world loaded. Can't simulate without a world!" << std::endl << std::flush;
        return;
    }

    if (continueSimulation) {
        std::cout << "Already simulating." << std::endl << std::flush;
        return;
    }

    // clear out parts of the timeline that are "in the future"
    // relative to the world that's about to start simulating, then
    // reset the timer's bounds
    if (timeVector.size() != 0) {
        std::vector<GRIPTimeSlice>::iterator curState = timeVector.begin();
        std::cout << "Timeline has " << timeVector.size() << " entries" << std::endl;
        while (curState->time < mWorld->mTime && curState != timeVector.end()) {
            curState++;
        }
        if (curState != timeVector.end()) {
            timeVector.erase(curState, timeVector.end());
        }
        std::cout << "Timeline has " << timeVector.size() << " entries" << std::endl;
    }
    tMax = mWorld->mTime;
    tIncrement = mWorld->mTimeStep;
    
    std::cout << "Simulating..." << std::endl << std::flush;
    continueSimulation = true;
    lastFrameTime = clock();
    simulationTime = clock();
    UpdateAndRedraw();

    // fire simulation start timestep hooks
    size_t numPages = tabView->GetPageCount();
    for(size_t i=0; i< numPages; i++) {
        GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	tab->GRIPEventSimulationStart();
    }

    // fire the event that tells us to start simulating
    int type = 0;
    wxCommandEvent evt(wxEVT_GRIP_SIMULATE_FRAME,GetId());
    evt.SetEventObject(this);
    evt.SetClientData((void*)&type);
    GetEventHandler()->AddPendingEvent(evt);
}

/**
 * @function OnSimulateSingle
 * @brief Run dynamic simulation for one frame
 * @date 2013-01-16
 */
void GRIPFrame::OnSimulateSingle(wxCommandEvent& event) {
    if (!mWorld) {
        std::cout << "No world loaded. Can't simulate without a world!" << std::endl << std::flush;
        return;
    }

    std::cout << "Simulating Single..." << std::endl;
    continueSimulation = true;
    SimulateFrame(event);
    continueSimulation = false;
}

/**
 * @function OnSimulateStop
 * @brief 
 * @date 2013-01-15
 */
void GRIPFrame::OnSimulateStop(wxCommandEvent& event) {
    if (!mWorld) {
        std::cout << "No world loaded. Can't do simulation things without a world!" << std::endl << std::flush;
        return;
    }

    // fire simulation start timestep hooks
    size_t numPages = tabView->GetPageCount();
    for(size_t i=0; i< numPages; i++) {
        GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	tab->GRIPEventSimulationStop();
    }

    continueSimulation = false;
    timeRelText->Clear();
    UpdateAndRedraw();
    clock_t stopTime = clock();
    std::cout << "Stopping simulation: "
    		  << ((float)(stopTime - simulationTime))/CLOCKS_PER_SEC << " seconds elapsed"
    		  << std::endl;
}

void GRIPFrame::SimulateFrame(wxCommandEvent& event) {
    if (!continueSimulation) { return; }

    size_t numPages = tabView->GetPageCount();

    // fire before timestep hooks
    for(size_t i=0; i< numPages; i++) {
        GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	tab->GRIPEventSimulationBeforeTimestep();
    }

    // Simulate a frame
    mWorld->step();

    // redraw if necessary
    if ((clock() - timeLastRedraw > (float)CLOCKS_PER_SEC/10.0)
        && renderDuringSimulation)
    {
        UpdateAndRedraw();
    }

    // fire after timestep hooks
    for(size_t i=0; i< numPages; i++) {
        GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	tab->GRIPEventSimulationAfterTimestep();
    }

    // save state to timeline
    AddWorld(mWorld);

    // calculate times - current time and how fast the simulation is
    // going. clock() has terrible resolution - upwards of ten
    // milliseconds - so we throw on a really crude kalman filter and
    // weight past evidence very highly.
    
    // TODO: Make a better initial estimate. Our current initial
    // estimate is hard-coded to be about right for our simple demo
    // scenes on a good desktop with slow framerate.
    
    // TODO: take rendering into account so we're only measuring
    // simulation time.
    
    // TODO: use simulation start and stop hook handlers to that
    // pausing doesn't mess with the estimate.
    double filterStrength = .995;
    clock_t curFrameTime = clock();
    double frameDuration = (float)(curFrameTime - lastFrameTime) / (float)CLOCKS_PER_SEC;
    double rawRelSimSpeed = frameDuration / mWorld->mTimeStep;
    lastFrameTime = curFrameTime;
    filteredRelSimSpeed = (filteredRelSimSpeed * filterStrength) + (rawRelSimSpeed * (1.0-filterStrength));
    
    // and then display the results
    timeText->Clear();
    timeText->SetInsertionPoint(0);
    timeText->AppendText(wxString::Format(wxT("%.3f"), mWorld->mTime)); // current time
    timeRelText->Clear();
    timeRelText->SetInsertionPoint(0);
    timeRelText->AppendText(wxString::Format(wxT("%.3f"), filteredRelSimSpeed)); // simulation speed

    // fire the event for the next simulation step. note that we use
    // AddPendingEvent to actually do fire an event here, making sure
    // that the rest of the UI gets its chance to do things. Just
    // calling the event handler would get us caught in a loop.
    wxYield();
    int type = 0;
    wxCommandEvent evt(wxEVT_GRIP_SIMULATE_FRAME,GetId());
    evt.SetEventObject(this);
    evt.SetClientData((void*)&type);
    GetEventHandler()->AddPendingEvent(evt);
}

/**
 * @function OnPlay 
 * @brief 
 * @date 2013-01-15
 */
void GRIPFrame::OnPlay(wxCommandEvent& event) {
    printf("OnPlay\n");
}

void GRIPFrame::OnRequestUpdateAndRender(wxCommandEvent& event) {
    UpdateAndRedraw();
}
void GRIPFrame::UpdateAndRedraw()
{
    for (int j = 0; j < mWorld->getNumRobots(); j++) {
        mWorld->getRobot(j)->update();
    }
    for (int j = 0; j < mWorld->getNumObjects(); j++) {
        mWorld->getObject(j)->update();
    }
    viewer->DrawGLScene();
    timeLastRedraw = clock();
}
// Must be called during the gl render
void GRIPFrame::FireEventRender()
{
    size_t numPages = tabView->GetPageCount();
    for(size_t i=0; i< numPages; i++) {
        GRIPTab* tab = (GRIPTab*)tabView->GetPage(i);
	tab->GRIPEventRender();
    }
}


BEGIN_EVENT_TABLE(GRIPFrame, wxFrame)
EVT_COMMAND_SCROLL(1009, GRIPFrame::OnTimeScroll)
EVT_TEXT_ENTER(1008, GRIPFrame::OnTimeEnter)

EVT_MENU(MenuSaveScene,  GRIPFrame::OnSaveScene)
EVT_MENU(MenuSaveRobot,  GRIPFrame::OnSaveRobot)
EVT_MENU(MenuLoad,  GRIPFrame::OnLoad)
EVT_MENU(MenuQuickLoad,  GRIPFrame::OnQuickLoad)
EVT_MENU(wxID_CLOSE,  GRIPFrame::OnClose)
EVT_MENU(MenuClose,  GRIPFrame::OnClose)
EVT_MENU(MenuQuit,  GRIPFrame::OnQuit)
EVT_MENU(MenuAbout, GRIPFrame::OnAbout)

EVT_MENU(MenuSimulateStart,  GRIPFrame::OnSimulateStart)
EVT_MENU(MenuSimulateStop, GRIPFrame::OnSimulateStop)
EVT_MENU(MenuSimulateSingle,  GRIPFrame::OnSimulateSingle)

EVT_MENU(MenuBgWhite,  GRIPFrame::OnWhite)
EVT_MENU(MenuBgBlack, GRIPFrame::OnBlack)
EVT_MENU(MenuRenderDuringSim, GRIPFrame::OnMenuRenderDuringSimulation)
EVT_MENU(MenuCameraReset, GRIPFrame::OnCameraReset)

EVT_MENU(MenuRenderVGA,  GRIPFrame::OnVGA)
EVT_MENU(MenuRenderXGA,  GRIPFrame::OnXGA)
EVT_MENU(MenuRenderHD, GRIPFrame::OnHD)

EVT_MENU(wxID_OPEN, GRIPFrame::OnLoad)
EVT_MENU(Tool_quickload, GRIPFrame::OnQuickLoad)
EVT_MENU(wxID_SAVE, GRIPFrame::OnSaveScene)

EVT_MENU(Tool_linkorder, GRIPFrame::OnToolOrder)
EVT_MENU(Tool_checkcollisions, GRIPFrame::OnToolCheckColl)
EVT_MENU(Tool_screenshot, GRIPFrame::OnToolScreenshot)
EVT_MENU(Tool_movie, GRIPFrame::OnToolMovie)

EVT_MENU(MenuRightSideView, GRIPFrame::OnViewChange)
EVT_MENU(MenuFrontView, GRIPFrame::OnViewChange)
EVT_MENU(MenuTopView, GRIPFrame::OnViewChange)
EVT_MENU(Tool_rightSideView, GRIPFrame::OnViewChange)
EVT_MENU(Tool_frontView, GRIPFrame::OnViewChange)
EVT_MENU(Tool_topView, GRIPFrame::OnViewChange)

EVT_TREE_SEL_CHANGED(TreeViewHandle,GRIPFrame::onTVChange)

EVT_COMMAND(wxID_ANY, wxEVT_GRIP_SIMULATE_FRAME, GRIPFrame::SimulateFrame)
EVT_COMMAND(wxID_ANY, wxEVT_GRIP_UPDATE_AND_RENDER, GRIPFrame::OnRequestUpdateAndRender)

//	EVT_BUTTON (BUTTON_Hello, GRIPFrame::OnQuit )
END_EVENT_TABLE()



