/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
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

#ifndef GRIP_GUI_H
#define GRIP_GUI_H

//--------------------------------------------------------------------
//   GLOBAL VARIABLES: Keep track of the user's current
//   loaded files, robot, planners, and other intentions w.r.t. the GUI
//---------------------------------------------------------------------

// FiGRIP some declarations of variable types

#include <wx/notebook.h>

class GRIPFrame;
class Viewer;
class TreeView;
class TreeViewReturn;
class InspectorTab;
class VisualizationTab;
class TemplateTab;
namespace robotics { class World; };

// The actual variables that are global to GUI elements

extern robotics::World  *mWorld;

extern int renderW,renderH,vgaW,vgaH,xgaW,xgaH,hd720W,hd720H;

extern GRIPFrame*	frame;
extern Viewer*		viewer;
extern TreeView*	treeView;
extern TreeViewReturn* selectedTreeNode;
extern wxNotebook*	tabView;


extern bool reverseLinkOrder;
extern bool check_for_collisions;


// Please don't change these constants - they are important to GUI functionality
static const int toolBarHeight = 30;
static const int prefViewerWidth = 644;
static const int prefViewerHeight = 484;
static const int prefTreeViewWidth = 210;
static const int prefTabsHeight = 200;
static const int statusBarHeight = 22;
static const double CAMERASPEED = 0.003f;

/* Tabs can issue this event to cause GRIP to start simulating */
DECLARE_EVENT_TYPE(wxEVT_GRIP_SIMULATE_FRAME, -1)

/* Tabs can issue this event to cause GRIP to render */
DECLARE_EVENT_TYPE(wxEVT_GRIP_UPDATE_AND_RENDER, -1)

/* Tabs can issue these events before and after they simulate to
 * notify other tabs that simulation is happening and that they need
 * to pay attention. A dynamics planner may not want to issue during
 * its planning, for example. */
DECLARE_EVENT_TYPE(wxEVT_GRIP_SIMULATION_START, -1)
DECLARE_EVENT_TYPE(wxEVT_GRIP_SIMULATION_END, -1)

/* Tabs can issue these events to notify other tabs that they are
 * starting or stopping the playback of a recording. */
DECLARE_EVENT_TYPE(wxEVT_GRIP_PLAYBACK_START, -1)
DECLARE_EVENT_TYPE(wxEVT_GRIP_PLAYBACK_END, -1)

/* Tabs can issue this event to notify other tabs that the world has
 * changed in some way. */
DECLARE_EVENT_TYPE(wxEVT_GRIP_WORLD_CHANGED, -1)

/* no idea what this event is really used for */
DECLARE_EVENT_TYPE(wxEVT_GRIP_STATE_CHANGE, -1)

#endif
