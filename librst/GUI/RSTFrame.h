//---------------------------------------------------------------------
//  Copyright (c) 2009 Mike Stilman
//  All Rights Reserved.
//
//  Permission to duplicate or use this software in whole or in part
//  is only granted by consultation with the author.
//
//    Mike Stilman              mstilman@cc.gatech.edu
//
//	  Robotics and Intelligent Machines
//    Georgia Tech
//--------------------------------------------------------------------

#ifndef RSTFRAME_H
#define RSTFRAME_H

class RSTSlider;
class Viewer;
class World;
class RSTimeSlice;
#include <vector>
#include "TreeView.h"

#include <string>
using namespace std;

#define Uses_wxThread
#include "wx/wxprec.h"
#ifndef WX_PRECOMP
	#include "wx/wx.h"
#endif
#include <wx/notebook.h>


// ----------------------------------------------------------------------------
class RSTFrame : public wxFrame
{
public:
    RSTFrame(const wxString& title);

	wxPanel *backPanel;

	RSTSlider *timeSlider;

	wxSlider *timeTrack;
	wxTextCtrl *timeText;

	wxToolBar* filebar;
	wxToolBar* optionbar;
	wxBitmap toolBarBitmaps[10];

	//void OnSize(wxSizeEvent& evt);
	int saveText(wxString scenepath, const char* llfile);
    void OnQuit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
    void OnSaveScene(wxCommandEvent& event);
    void OnSaveRobot(wxCommandEvent& event);
	void OnLoad(wxCommandEvent& event);
	void OnQuickLoad(wxCommandEvent& event);
	void OnToolOrder(wxCommandEvent& event);
	void OnToolCheckColl(wxCommandEvent& event);
	void OnToolScreenshot(wxCommandEvent& event);
	void OnToolMovie(wxCommandEvent& event);
    void OnClose(wxCommandEvent& event);

	void OnTimeScroll(wxScrollEvent &evt);
	void OnTimeEnter(wxCommandEvent &evt);

	void OnWhite(wxCommandEvent& event);
	void OnBlack(wxCommandEvent& event);
	void OnCameraReset(wxCommandEvent& event);

	void InitTimer(string title, double period);
	void AddWorld(World* world);

	vector<RSTimeSlice*> timeVector;
	double tCurrent;
	double tMax;
	double tIncrement;
	int tPrecision;
	void setTimeValue(double value, bool sendSignal = false);
	void updateTimeValue(double value, bool sendSignal = false);

	void updateAllTabs();

	void DoLoad(string filename);
	void DeleteWorld();

	void onTVChange(wxTreeEvent& event);

    DECLARE_EVENT_TABLE()
};


enum
{
	MenuSaveScene = wxID_HIGHEST+1,
	MenuSaveRobot,
	MenuLoad,
	MenuQuickLoad,
	MenuClose,
	MenuBgWhite,
	MenuBgBlack,
	MenuCameraReset,
    MenuQuit = wxID_EXIT,
    MenuAbout = wxID_ABOUT
};



#endif
