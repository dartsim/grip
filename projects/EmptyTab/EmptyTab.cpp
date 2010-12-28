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
#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/RSTSlider.h>
#include <GUI/RSTFrame.h>
#include <iostream>
using namespace std;
#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/Link.h>
#include <Tools/Object.h>
#include <Tools/Constants.h>


#include "EmptyTab.h"
#include <Tabs/AllTabs.h>
#include <RSTApp.h>


// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum EmptyTabEvents {
};
//Add a handlers for UI changes
BEGIN_EVENT_TABLE(EmptyTab, wxPanel)
END_EVENT_TABLE ()
// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(EmptyTab, RSTTab)
EmptyTab::EmptyTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) : RSTTab(parent, id, pos, size, style) {
}

// All tabs get a message for certain changes in RST (in case they want to do something)
void EmptyTab::RSTStateChange() {
}

// RST Library Calls THIS function to add your tab (put initialization here)
void RSTApp::AddTab() {
	ADD_TAB(EmptyTab,wxT("EmptyTab"));
}


