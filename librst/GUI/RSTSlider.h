//---------------------------------------------------------------------
//  Copyright (c) 2009 Mike Stilman & Saul Reynolds-Haertle
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

#ifndef RSTSLIDER_H
#define RSTSLIDER_H

#include <wx/wx.h>
#include <string>

using namespace std;

DECLARE_EVENT_TYPE(wxEVT_RST_SLIDER_CHANGE, -1)

class RSTSlider : public wxPanel
{
public:
	RSTSlider(){}
	RSTSlider(wxBitmap bmp, double left, double right, int precision, double initialpos,
						int lineSize, int pageSize,
						wxWindow *parent, const wxWindowID id = -1, bool vertical = false,
                       const wxPoint& pos = wxDefaultPosition, 
					   const wxSize& size = wxDefaultSize,
                       long style = wxTAB_TRAVERSAL);
	RSTSlider(const char* name, double left, double right, int precision, double initialpos,
						int lineSize, int pageSize,
						wxWindow *parent, const wxWindowID id = -1, bool vertical = false,
                       const wxPoint& pos = wxDefaultPosition, 
					   const wxSize& size = wxDefaultSize,
                       long style = wxTAB_TRAVERSAL);
	virtual ~RSTSlider(){}

	wxBoxSizer *sizer;

	wxSlider *track;
	wxStaticText *lText;
	wxTextCtrl *rText;
	wxStaticBitmap *bmpButton;

	//string name;
	double pos;
	double leftBound;
	double rightBound;
	double tickfrequency;
	int prec;

	void setRange(double left, double right);
	void setValue(double value, bool sendSignal = true);
	void updateValue(double value, bool sendSignal = true);
	void setPrecision(int precision);

	void OnScroll(wxScrollEvent &evt);
	void OnEnter(wxCommandEvent &evt);


	DECLARE_DYNAMIC_CLASS(RSTSlider)
	DECLARE_EVENT_TABLE()
};

#endif