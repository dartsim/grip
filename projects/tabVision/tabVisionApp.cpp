/**
 * @file tabVisionApp.cpp
 * @author Can Erdogan
 * @date Nov 27, 2012
 * @brief The main application interface for the tab which contains: "IMPLEMENT_APP".
 */

#include "GRIPApp.h"
#include "tabVision.h"

extern wxNotebook* tabView;

class VisionTabApp : public GRIPApp {
	virtual void AddTabs() {
		tabView->AddPage(new VisionTab(tabView), wxT("Vision"));
	}
};

IMPLEMENT_APP(VisionTabApp)

