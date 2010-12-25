//---------------------------------------------------------------------
//  Copyright (c) 2008 Saul Reynolds-Haertle & Mike Stilman
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
#include "GUI.h"
//--------------------------------------------------------------------
//   GLOBAL VARIABLES: Keep track of the user's current
//   loaded files, robot, planners, and other intentions w.r.t. the GUI
//---------------------------------------------------------------------

World*		world=0;
Robot*		robot=0;
Trajectory*	trajectory=0;

RSTFrame*	frame=0;
Viewer*		viewer=0;
TreeView*	treeView=0;
wxNotebook*	tabView=0;

TreeViewReturn* selectedTreeNode=0;

bool reverseLinkOrder = false;
bool check_for_collisions = false;
int stateChangeType = 0;

DEFINE_EVENT_TYPE(wxEVT_RST_STATE_CHANGE)

