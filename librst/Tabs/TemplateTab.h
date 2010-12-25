//---------------------------------------------------------------------
//  Copyright (c) 2009 Mike Stilman
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

#ifndef TEMPLATE_TAB
#define TEMPLATE_TAB

#include "RSTTab.h"
#include "RSTThread.h"

class TemplateTab : public RSTTab
{
public:
	TemplateTab(){};
    TemplateTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
             long style = wxTAB_TRAVERSAL);
	virtual ~TemplateTab(){}

	wxStaticText* sampleText1;
	wxStaticText* sampleText2;

	wxSizer* sizerFull;
	RSTSlider* sampleRSTSlider1;
	RSTSlider* sampleRSTSlider2;

	void OnSlider(wxCommandEvent &evt);
	void RSTStateChange();

	// Thread specific
	// RSTThread* thread;

	// Your Thread routine
	// call RSTThread::CheckPoint() regularly
	// void Thread();
	// void onCompleteThread();

	DECLARE_DYNAMIC_CLASS(TemplateTab)
	DECLARE_EVENT_TABLE()
};

#endif
