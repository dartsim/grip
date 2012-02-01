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

#ifndef TEMPLATE_TAB
#define TEMPLATE_TAB

#include <Tabs/GRIPTab.h>
#include <Tabs/GRIPThread.h>
#include <Tools/Constants.h>
#include "GRIPServer.h"
#include <Tools/3dbpp.h>

class PalletizingTab : public GRIPTab
{

	GRIPServer* server;

public:
	PalletizingTab(){};
    PalletizingTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~PalletizingTab(){
		delete server;
	}

	wxStaticText* text1;
	wxStaticText* text2;
	wxStaticText* text3;
	wxSizer* sizerFull;

	GRIPThread* thread;

	// call RSTThread::CheckPoint() regularly
	void Thread();
	void onCompleteThread();

	void OnButton(wxCommandEvent &evt);
	void GRIPStateChange();
	void setNumOfLayers(int);
	void setNumOfPallets(int);
	void setCost(const char*, float);

	DECLARE_DYNAMIC_CLASS(PalletizingTab)
	DECLARE_EVENT_TABLE()
};


#endif
