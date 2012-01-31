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

#include "tabPalletizing.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>
using namespace std;

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>


//Give each slider a number so we recognize them (also indicates order of select on tabbing)
enum sliderNames {
	SAMPLE_GRIP_SLIDER1 = 1000, SAMPLE_GRIP_SLIDER2 = 1001
};

//Add a handler for slider changes
BEGIN_EVENT_TABLE(PalletizingTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, PalletizingTab::OnSlider)
EVT_MENU(MenuPalletLoad,  PalletizingTab::OnPalletLoad)
END_EVENT_TABLE ()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(PalletizingTab, GRIPTab)

PalletizingTab::PalletizingTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) :
	GRIPTab(parent, id, pos, size, style) {
	sizerFull = new wxBoxSizer(wxHORIZONTAL);

	// Create Static boxes - these are the outlines you see on the inspector tab - a nice way to organize things
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Sample Box 1"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Sample Box 2"));

	// Create sizers for these static boxes
	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);

	// Add 2 static text fields (these can be re-written by the handler)
	sampleText1 = new wxStaticText(this, -1, wxT("Sample text 1"),
			wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);
	sampleText2 = new wxStaticText(this, -1, wxT("Sample text 2"),
			wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);

	// Create GRIP-style sliders
	sampleGRIPSlider1 = new GRIPSlider("SS1", -180, 180, 2000, 0, 1000, 2000,
			this, SAMPLE_GRIP_SLIDER1);
	sampleGRIPSlider2 = new GRIPSlider("SS2", -180, 180, 2000, 0, 1000, 2000,
			this, SAMPLE_GRIP_SLIDER2);

	// Add the boxes to their respective sizers
	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	SetSizer(sizerFull);

	// Add content to box1 (1st sample text and slider)
	ss1BoxS->Add(sampleText1, 1, wxEXPAND | wxALL, 6);
	ss1BoxS->Add(sampleGRIPSlider1, 1, wxEXPAND | wxALL, 6);

	// Add content to box2 (2nd sample text and slider)
	ss2BoxS->Add(sampleGRIPSlider2, 1, wxEXPAND | wxALL, 6);
	ss2BoxS->Add(sampleText2, 1, wxEXPAND | wxALL, 6);

	wxMenu *palletMenu = new wxMenu;
	palletMenu->Append(MenuPalletLoad, wxT("L&oad Palletizing Files"));

	wxMenuBar *menuBar = frame->GetMenuBar();
	menuBar->Append(palletMenu, wxT("&Palletizing"));
	frame->SetMenuBar(menuBar);

//	server = new GRIPServer();
//	server->setup();
//
//	thread = new GRIPThread(this);
//	gripThreadErrorToString(thread->CreateThread());
}

//Handle slider changes
void PalletizingTab::OnSlider(wxCommandEvent &evt) {
	if(selectedTreeNode==NULL){
		return;
	}

	int slnum = evt.GetId();
	double pos = *(double*) evt.GetClientData();
	char numBuf[64];
    numBuf[0] = '\0';
	//sprintf(numBuf, "");

	switch (slnum) {
	case SAMPLE_GRIP_SLIDER1:
		cout << "Changing slider 1" << endl;
		sprintf(numBuf, "X Change: %7.4f", pos);
		break;
	case SAMPLE_GRIP_SLIDER2:
		cout << "Changing slider 2" << endl;
		sprintf(numBuf, "Y Change: %7.4f", pos);
		break;

	default:
		return;
	}
//cout << "got here" << endl;
	//world->updateCollision(o);
//	viewer->UpdateCamera();

	if (frame != NULL)
		frame->SetStatusText(wxString(numBuf, wxConvUTF8));
}


// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void PalletizingTab::GRIPStateChange() {
	if(selectedTreeNode==NULL){
		return;
	}

	string statusBuf;
	string buf, buf2;
	switch (selectedTreeNode->dType) {
	case Return_Type_Object:
		statusBuf = " Selected Object: ";
		buf = "You clicked on object: ";
		sampleText1->SetLabel(wxString(buf.c_str(), wxConvUTF8));
		sampleText2->Hide();

		break;
	case Return_Type_Robot:
		statusBuf = " Selected Robot: ";
		buf = "You clicked on robot: ";
		sampleText2->SetLabel(wxString(buf.c_str(), wxConvUTF8));
		sampleText1->Hide();

		break;
	case Return_Type_Node:
		statusBuf = " Selected Link:  of Robot: ";
		buf = " Link:  of Robot: ";
		// Do something here if you want to.  you get the idea...

		break;
    default:
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
	}
	//frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
	//sizerFull->Layout();
}


void PalletizingTab::Thread() {
	while(1) {
		server->acceptMode();
	}
}

void PalletizingTab::onCompleteThread() {
	printf("Not accepting any more connections\n");
}

void PalletizingTab::OnPalletLoad(wxCommandEvent& WXUNUSED(event)) {

}
