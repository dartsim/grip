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

#ifndef PLANNING_TAB
#define PLANNING_TAB

#include "GRIPTab.h"
#include <Eigen/Core>

class PlanningTab : public GRIPTab
{
public:
	PlanningTab(){};
    PlanningTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~PlanningTab(){}

	int rrtStyle;
	bool greedyMode;
	bool connectMode;
	bool showProg;

	int robotID;

	wxTextCtrl *timeText;

	Eigen::VectorXd startConf;
	Eigen::VectorXd goalConf;


	void OnSlider(wxCommandEvent &evt);
	void OnRadio(wxCommandEvent &evt);
	void OnButton(wxCommandEvent &evt);
	void OnCheckBox(wxCommandEvent &evt);
	void SetTimeline(int robot, std::vector<int> links, std::list<Eigen::VectorXd> path);
	void GRIPStateChange();

	DECLARE_DYNAMIC_CLASS(PlanningTab)
	DECLARE_EVENT_TABLE()
};

#endif
