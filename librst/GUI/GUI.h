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

#ifndef GUI_H
#define GUI_H

//--------------------------------------------------------------------
//   GLOBAL VARIABLES: Keep track of the user's current
//   loaded files, robot, planners, and other intentions w.r.t. the GUI
//---------------------------------------------------------------------

// First some declarations of variable types

#include <wx/notebook.h>

//class Planner;
class Trajectory;
class Robot;
class World;

class RSTFrame;
class Viewer;
class TreeView;
class TreeViewReturn;
class InspectorTab;
class TemplateTab;

// The actual variables that are global to GUI elements

extern World*		world;
extern Robot*		robot;
extern Trajectory*	trajectory;

extern RSTFrame*	frame;
extern Viewer*		viewer;
extern TreeView*	treeView;
extern wxNotebook*	tabView;

extern TreeViewReturn* selectedTreeNode;

extern bool reverseLinkOrder;
extern bool check_for_collisions;

// Please don't change these constants - they are important to GUI functionality
static const int toolBarHeight = 30;

//#ifdef __APPLE__
//static const int prefViewerWidth = 644;
//#else
static const int prefViewerWidth = 644;
//#endif
static const int prefViewerHeight = 484;
static const int prefTreeViewWidth = 210;
static const int prefTabsHeight = 200;
static const int statusBarHeight = 22;
static const double defaultCamRadius = 10.f;
static const double CAMERASPEED = 0.003f;


DECLARE_EVENT_TYPE(wxEVT_RST_STATE_CHANGE, -1)

#endif
