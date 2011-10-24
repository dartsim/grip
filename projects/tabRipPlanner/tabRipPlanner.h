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

#ifndef RIP_PLANNING_TAB
#define RIP_PLANNING_TAB

#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>

#include <kinematics/BodyNode.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <planning/Robot.h>
#include <planning/Object.h>

#include <Tabs/GRIPThread.h>
#include <Tools/Constants.h>

//#include <Matrix.h>
#include <planning/Robot.h>
//#include "../Tools/Planner.h"

#define rstate vector<double>

using namespace std;
using namespace planning;

class RipTabPlanning : public GRIPTab
{
public:
	RipTabPlanning(){};
    RipTabPlanning(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~RipTabPlanning(){}

	int rrtStyle;
	bool greedyMode;
	bool connectMode;
	bool showProg;
	//Planner *planner;

	int robotID;

	wxTextCtrl *timeText;

	rstate startConf;
	rstate goalConf;

	// public vars to capture external selection stuff (should move these higher somethime)
	//Object* selectedObject;
	//Robot* selectedRobot;
	//Link* selectedLink;

	void OnSlider(wxCommandEvent &evt);
	void OnRadio(wxCommandEvent &evt);
	void OnButton(wxCommandEvent &evt);
	void OnCheckBox(wxCommandEvent &evt);
	void SetTimeline();
	void GRIPStateChange();

	// Thread specific
	// GRIPThread* thread;

	// Your Thread routine
	// call GRIPThread::CheckPoint() regularly
	// void Thread();
	// void onCompleteThread();

	DECLARE_DYNAMIC_CLASS(RipTabPlanning)
	DECLARE_EVENT_TABLE()
};

#endif
