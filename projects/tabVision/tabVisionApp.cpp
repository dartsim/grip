/**
 * @file tabVisionApp.cpp
 * @author Can Erdogan
 * @date Nov 27, 2012
 * @brief The main application interface for the tab which contains: "IMPLEMENT_APP".
 */

#include "GRIPApp.h"
#include "tabVision.h"
#include <GUI/GUI.h>
#include <GUI/GRIPFrame.h>

extern wxNotebook* tabView;

class VisionTabApp : public GRIPApp {

	/// Add the vision tab
	virtual void AddTabs() {
		tabView->AddPage(new VisionTab(tabView), wxT("Vision"));
	}

	/// Initialize the frame and then load a scene
	virtual bool OnInit() {

		// Create the app
		bool success = GRIPApp::OnInit();
	
		// Load the frame
		frame->DoLoad("../scenes/DesktopArmCamera.urdf");
		return true;
	}
};

IMPLEMENT_APP(VisionTabApp)

