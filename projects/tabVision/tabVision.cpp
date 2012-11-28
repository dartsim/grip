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


#include "tabVision.h"

#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
using namespace std;

enum VisionTabEvents {
	button_attention = 50, 
  button_go,
  button_,
  button_showGoal
};

void VisionTab::OnButton(wxCommandEvent &evt) {

}


VisionTab::VisionTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) : GRIPTab(parent, id, pos, size, style) {

	// ===========================================================
	// 1. Create the left side for the vision demonstration

	// Create StaticBox container for the two buttons: "Attention!" and "Go!"
	wxStaticBox* leftBox = new wxStaticBox(this, -1, wxT("Demonstration"));
	wxStaticBoxSizer* leftBoxSizer = new wxStaticBoxSizer(leftBox, wxVERTICAL);

	// Add the "Attention!" button
	leftBoxSizer->Add(new wxButton(this, button_attention, wxT("Attention!")), 0, wxALL, 10);

	// Add the "Go!" button
	leftBoxSizer->Add(new wxButton(this, button_go, wxT("Start Search!")), 0, wxALL, 10);

	// ===========================================================
	// 2. Create the right side for 3D data acquisition

	// Create StaticBox container for the two buttons: "Show Cloud" and "Show Depth Map"
	wxStaticBox* rightBox = new wxStaticBox(this, -1, wxT("3D Data"));
	wxStaticBoxSizer* rightBoxSizer = new wxStaticBoxSizer(rightBox, wxVERTICAL);

	// Add the "Attention!" button
	rightBoxSizer->Add(new wxButton(this, button_cloud, wxT("Show Cloud")), 0, wxALL, 10);

	// Add the "Go!" button
	rightBoxSizer->Add(new wxButton(this, button_depthmap, wxT("Show Depth Map")), 0, wxALL, 10);

	// ===========================================================
	// 3. Create empty far right container to look nice
	wxStaticBox* emptyBox = new wxStaticBox(this, -1, wxT(""));
	wxStaticBoxSizer* emptyBoxSizer = new wxStaticBoxSizer(emptyBox, wxVERTICAL);

	// ===========================================================
	// 4. Add both sizers to the full sizer

	// Create the sizer that controls the tab panel
	wxBoxSizer* sizerFull = new wxBoxSizer (wxHORIZONTAL);
	
	// Add the sides
	sizerFull->Add(leftBoxSizer, 2, wxALL | wxEXPAND, 10);
	sizerFull->Add(rightBoxSizer, 2, wxALL | wxEXPAND, 10);
	sizerFull->Add(emptyBoxSizer, 5, wxALL | wxEXPAND, 10);

	// Set the full sizer as the sizer of this tab
	SetSizer(sizerFull);

}

//Add a handlers for UI changes
BEGIN_EVENT_TABLE(VisionTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, VisionTab::OnButton)
END_EVENT_TABLE ()

// Class constructor for the tab
IMPLEMENT_DYNAMIC_CLASS(VisionTab, GRIPTab)


