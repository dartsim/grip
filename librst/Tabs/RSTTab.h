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

#ifndef RST_TAB_H
#define RST_TAB_H

#include <wx/wx.h>

#include <GUI/RSTSlider.h>
#include <GUI/TreeView.h>

class RSTTab : public wxPanel
{
public:
	RSTTab(){};
    RSTTab(wxWindow * parent, wxWindowID id = -1,
             const wxPoint & pos = wxDefaultPosition,
             const wxSize & size = wxDefaultSize,
			 long style = wxTAB_TRAVERSAL) : wxPanel(parent, id, pos, size, style) {};
	virtual ~RSTTab(){}

	virtual void RSTStateChange(){};

	// call RSTThread::CheckPoint() regularly
	virtual void Thread() {};
	virtual void onThreadComplete() {};

};

#endif
