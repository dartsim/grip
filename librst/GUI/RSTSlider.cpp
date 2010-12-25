//---------------------------------------------------------------------
//  Copyright (c) 2009  Mike Stilman & Saul Reynolds-Haertle
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
#include "RSTSlider.h"
#include <string>
#include <iostream>
using namespace std;

BEGIN_EVENT_TABLE(RSTSlider, wxPanel)
	EVT_COMMAND_SCROLL(1009, RSTSlider::OnScroll)
	EVT_TEXT_ENTER(1008, RSTSlider::OnEnter)
END_EVENT_TABLE()

IMPLEMENT_DYNAMIC_CLASS(RSTSlider, wxPanel)
DEFINE_EVENT_TYPE(wxEVT_RST_SLIDER_CHANGE)

RSTSlider::RSTSlider(const char* name, double left, double right, int precision, double initialpos,
					 int lineSize, int pageSize,
					 wxWindow *parent, const wxWindowID id, bool vertical,
                       const wxPoint& pos, const wxSize& size,
                       long style)
					   : wxPanel(parent, id, pos, size, style)
{
	SetAutoLayout(true);

	if(vertical){
		sizer = new wxBoxSizer(wxVERTICAL);
		lText = new wxStaticText(this,-1,wxString(name,wxConvUTF8),wxDefaultPosition,wxDefaultSize,wxALIGN_CENTRE); //wxSize(40,40)
		rText = new wxTextCtrl(this,1008,wxT("0.00"),wxDefaultPosition,wxSize(40,20),wxTE_PROCESS_ENTER | wxTE_RIGHT);

		track = new wxSlider(this,1009,0,-100,100,wxDefaultPosition, wxSize(40,100), wxSL_BOTH | wxSL_VERTICAL | wxALIGN_CENTRE);
		track->SetLineSize(lineSize);
		track->SetPageSize(pageSize);

		sizer->Add(lText, 0, wxEXPAND | wxALIGN_CENTER_HORIZONTAL  | wxALL, 2);
		sizer->Add(track, 1, wxEXPAND | wxLEFT , 10);
		sizer->Add(rText, 0, wxALIGN_CENTER_HORIZONTAL | wxRIGHT, 2);
	}else{
		sizer = new wxBoxSizer(wxHORIZONTAL);
		lText = new wxStaticText(this,-1,wxString(name,wxConvUTF8),wxDefaultPosition,wxDefaultSize,wxALIGN_CENTRE); //wxSize(40,40)
		rText = new wxTextCtrl(this,1008,wxT("  0.00"),wxDefaultPosition,wxSize(50,20),wxTE_PROCESS_ENTER | wxTE_RIGHT);

		track = new wxSlider(this,1009,0,-100,100,wxDefaultPosition, wxSize(100,20), wxSL_BOTH);
		track->SetLineSize(lineSize);
		track->SetPageSize(pageSize);

		sizer->Add(lText, 0, wxEXPAND | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL  | wxALL, 6);
		sizer->Add(track, 1, wxEXPAND | wxALL | wxALIGN_CENTER_VERTICAL, 6);
		sizer->Add(rText, 0, wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL  | wxALL, 6);
	}

	setPrecision(precision);
	setRange(left,right);
	setValue(initialpos);

	SetSizer(sizer);
}

RSTSlider::RSTSlider(wxBitmap bmp, double left, double right, int precision, double initialpos,
					 int lineSize, int pageSize,
					 wxWindow *parent, const wxWindowID id, bool vertical,
                       const wxPoint& pos, const wxSize& size,
                       long style)
					   : wxPanel(parent, id, pos, size, style)
{
	SetAutoLayout(true);

	if(vertical){
		sizer = new wxBoxSizer(wxVERTICAL);
		bmpButton = new wxStaticBitmap(this, -1, bmp);
		rText = new wxTextCtrl(this,1008,wxT("0.00"),wxDefaultPosition,wxSize(40,20),wxTE_PROCESS_ENTER | wxTE_RIGHT);

		track = new wxSlider(this,1009,0,-100,100,wxDefaultPosition, wxSize(10,100), wxSL_BOTH | wxSL_VERTICAL | wxALIGN_CENTRE);
		track->SetLineSize(lineSize);
		track->SetPageSize(pageSize);

		sizer->Add(bmpButton, 0, wxEXPAND | wxALIGN_CENTER_HORIZONTAL  | wxALL, 0);
		sizer->Add(track, 1, wxEXPAND | wxALL , 2);
		sizer->Add(rText, 0, wxALIGN_CENTER_HORIZONTAL | wxALL, 2);
		rText->Hide();
	}else{
		sizer = new wxBoxSizer(wxHORIZONTAL);
		bmpButton = new wxStaticBitmap(this, -1, bmp);
		//bmpButton = new wxBitmapButton(this, -1, bmp);
		//bmpButton->Disable();
		rText = new wxTextCtrl(this,1008,wxT(" 0.00"),wxDefaultPosition,wxSize(50,20),wxTE_PROCESS_ENTER | wxTE_RIGHT);

		track = new wxSlider(this,1009,0,-100,100,wxDefaultPosition, wxSize(100,40), wxSL_BOTH);
		track->SetLineSize(lineSize);
		track->SetPageSize(pageSize);

		sizer->Add(bmpButton, 0, wxEXPAND | wxALIGN_CENTER_HORIZONTAL  | wxALL, 2);
//		sizer->Add(lText, 0, wxEXPAND | wxALIGN_LEFT | wxALIGN_CENTER_VERTICAL  | wxALL, 6);
		sizer->Add(track, 1, wxEXPAND | wxALL | wxALIGN_CENTER_VERTICAL, 6);
		sizer->Add(rText, 0, wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL  | wxALL, 6);
	}

	setPrecision(precision);
	setRange(left,right);
	setValue(initialpos);

	SetSizer(sizer);
}



void RSTSlider::setPrecision(int precision){
	prec = precision;
}

void RSTSlider::setRange(double left, double right){
	leftBound = left;
	rightBound = right;
	track->SetRange(leftBound * prec, rightBound * prec);
}

void RSTSlider::setValue(double value, bool sendSignal){
	pos = value;
	track->SetValue(value * prec);
	updateValue(value, sendSignal);
}

void RSTSlider::updateValue(double value, bool sendSignal){
	char buf[100];
	sprintf(buf, "%6.2f", pos);
	wxString posString = wxString(buf,wxConvUTF8);
	rText->ChangeValue(posString);

	//Send a slider change event up to the parent window
	if(sendSignal){
		wxCommandEvent evt(wxEVT_RST_SLIDER_CHANGE,GetId());
		evt.SetEventObject(this);
		evt.SetClientData((void*)&pos);
		GetEventHandler()->ProcessEvent(evt);
	}
}

void RSTSlider::OnScroll(wxScrollEvent &evt){
	pos = (double)(evt.GetPosition())/(double)prec;
	updateValue(pos);
}

void RSTSlider::OnEnter(wxCommandEvent& WXUNUSED(evt)){
	double p;
	rText->GetValue().ToDouble(&p);
	if(p < leftBound) p = leftBound;
	if(p > rightBound) p = rightBound;
	setValue(p);
}
