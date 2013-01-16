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

#ifndef GRIP_TAB_H
#define GRIP_TAB_H

#include <wx/wx.h>

#include <GUI/GRIPSlider.h>
#include <GUI/TreeView.h>

class GRIPTab : public wxPanel
{
public:
	GRIPTab(){};
    GRIPTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
			 long style = wxTAB_TRAVERSAL) : wxPanel(parent, id, pos, size, style) {};
	virtual ~GRIPTab(){}


        /* general event */
        /* TODO: learn more about this */
	virtual void GRIPStateChange(){};

        /* Fire in relation to dyanmic simulation */
        /* Suitable for controllers */
        virtual void GRIPEventSimulationBeforeTimestep(){};
        virtual void GRIPEventSimulationAfterTimestep(){};
        virtual void GRIPEventSimulationStart(){};
        virtual void GRIPEventSimulationEnd(){};

        /* Fire in relation to movie playback/history slider traversal */
        /* Suitable for displaying graphical effects or object
         * information */
        virtual void GRIPEventPlaybackBeforeFrame(){};
        virtual void GRIPEventPlaybackAfterFrame(){};
        virtual void GRIPEventPlaybackStart(){};
        virtual void GRIPEventPlaybackEnd(){};

        /* fires when the world is changed by something other than the
         * dyanmics simulation, like the inspector tab */
        virtual void GRIPEventWorldChanged(){};
        
        /* fires when a new object is selected in the treeview */
        virtual void GRIPEventTreeViewSelectionChanged(){};
        
        /* fires when worlds are loaded or unloaded */
        virtual void GRIPEventSceneLoaded(){};
        virtual void GRIPEventSceneUnloaded(){};

        /* fires during render so tabs can add objects to the world */
        virtual void GRIPEventRender(){};

	// call GRIPThread::CheckPoint() regularly
	virtual void Thread() {};
	virtual void onThreadComplete() {};

};

#endif
