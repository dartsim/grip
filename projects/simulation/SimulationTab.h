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

#ifndef SIMULATION_TAB
#define SIMULATION_TAB

#include <Tabs/RSTTab.h>

class SimulationTab : public RSTTab
{
public:
	SimulationTab(){};
    SimulationTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~SimulationTab(){}

	wxSizer* sizerFull;

	void OnButton(wxCommandEvent &evt);
	void RSTStateChange();

	DECLARE_DYNAMIC_CLASS(SimulationTab)
	DECLARE_EVENT_TABLE()
};

#endif
