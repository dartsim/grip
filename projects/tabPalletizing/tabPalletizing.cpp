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
#include <wx/dirdlg.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <iostream>
using namespace std;

#include <Tabs/AllTabs.h>
#include <GRIPApp.h>

enum buttonEvents {
	be_refresh = 50
};


BEGIN_EVENT_TABLE(PalletizingTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, PalletizingTab::OnButton)
END_EVENT_TABLE ()


IMPLEMENT_DYNAMIC_CLASS(PalletizingTab, GRIPTab)

PalletizingTab::PalletizingTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) :
	GRIPTab(parent, id, pos, size, style) {
	sizerFull = new wxBoxSizer(wxHORIZONTAL);

	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Info"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT("Control"));

	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxVERTICAL);

	text1 = new wxStaticText(this, -1, wxT(""),
			wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);
	text2 = new wxStaticText(this, -1, wxT(""),
			wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);
	text3 = new wxStaticText(this, -1, wxT(""),
			wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE);
	setNumOfLayers(0);
	setNumOfPallets(0);
	setCost("None selected", 0.0f);
	wxButton* button_refresh = new wxButton(this, be_refresh, wxT("Refresh"));

	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss2BoxS, 3, wxEXPAND | wxALL, 6);
	SetSizer(sizerFull);

	ss1BoxS->Add(text1, 1, wxEXPAND | wxALL, 6);
	ss1BoxS->Add(text2, 1, wxEXPAND | wxALL, 6);
	ss1BoxS->Add(button_refresh, 3, wxEXPAND | wxALL, 6);

	ss2BoxS->Add(text3, 1, wxEXPAND | wxALL, 6);

	//	server = new GRIPServer();
	//	server->setup();
	//
	//	thread = new GRIPThread(this);
	//	gripThreadErrorToString(thread->CreateThread());
}

void PalletizingTab::OnButton(wxCommandEvent &evt) {
	int button_num = evt.GetId();
	switch (button_num) {
	case be_refresh:
		setNumOfLayers(d.layer_map.size());
		break;
	}
}

// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void PalletizingTab::GRIPStateChange() {
	if (selectedTreeNode == NULL) {
		return;
	}

	config_t *c;
	vector<config_t>* p;
	string buf;

	switch (selectedTreeNode->dType) {
	case Return_Type_Config:
		c = reinterpret_cast<config_t*> (selectedTreeNode->data);
		configCurrent = *c;
		buf = "Selected " + configCurrent.key_s();
		setCost(configCurrent.key_s().c_str(), (float) configCurrent.get_weight());
		keyChanged = 1;
		viewer->ResetGL();
		break;
	case Return_Type_Packlist:
		p = reinterpret_cast<packlist_*> (selectedTreeNode->data);
		packlistCurrent = *p;
		buf = "Selected Packlist";
		keyChanged = 2;
		viewer->ResetGL();
		break;
	default:
		buf = "Object of unknown data type selected";
		break;
	}

	frame->SetStatusText(wxString(buf.c_str(), wxConvUTF8));
	sizerFull->Layout();
}

void PalletizingTab::Thread() {
	while (1) {
		server->acceptMode();
	}
}

void PalletizingTab::onCompleteThread() {
	printf("Not accepting any more connections\n");
}

void PalletizingTab::setNumOfLayers(int nlayers) {
	char buf_layers[100];
	sprintf(buf_layers, "Num. layers = %d", nlayers);
	text1->SetLabel(wxString(buf_layers, wxConvUTF8));
}

void PalletizingTab::setNumOfPallets(int npallets) {
	char buf_layers[100];
	sprintf(buf_layers, "Num. pallets = %d", npallets);
	text2->SetLabel(wxString(buf_layers, wxConvUTF8));
}

void PalletizingTab::setCost(const char* str, float _cost) {
	char buf_layers[100];
	sprintf(buf_layers, "Cost of %s is %.2f", str, _cost);
	text3->SetLabel(wxString(buf_layers, wxConvUTF8));
}
