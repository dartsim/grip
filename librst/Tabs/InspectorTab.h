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


#ifndef INSPECTOR_TAB
#define INSPECTOR_TAB

#include <Tabs/RSTTab.h>
#include <Tools/Robot.h>
#include <Tools/World.h>
#include <Tools/Link.h>
#include <Tools/Object.h>
#include <Tools/Constants.h>


class InspectorTab : public RSTTab
{
public:
	InspectorTab(){};
    InspectorTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~InspectorTab(){}

	wxStaticText* itemName;
	wxStaticText* parentName;
/* 	wxSlider* jointSlider; */

	wxSizer* sizerFull;
	RSTSlider* jSlider;
	RSTSlider* xSlider;
	RSTSlider* ySlider;
	RSTSlider* zSlider;
	RSTSlider* rollSlider;
	RSTSlider* pitchSlider;
	RSTSlider* yawSlider;

	void OnSlider(wxCommandEvent &evt);
	void RSTStateChange();

	DECLARE_DYNAMIC_CLASS(InspectorTab)
	DECLARE_EVENT_TABLE()
};

#endif
