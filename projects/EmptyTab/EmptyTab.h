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

#ifndef EMPTY_TAB
#define EMPTY_TAB

#include <Tabs/RSTTab.h>
#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/Link.h>
#include <Tools/Object.h>
#include <Tools/Constants.h>

class EmptyTab : public RSTTab
{
public:
	EmptyTab(){};
    EmptyTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~EmptyTab(){}

	void RSTStateChange();

	DECLARE_DYNAMIC_CLASS(EmptyTab)
	DECLARE_EVENT_TABLE()
};

#endif
