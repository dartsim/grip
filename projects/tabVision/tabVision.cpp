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
#include <GUI/Camera.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>

#include <robotics/World.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

#include <fstream>

using namespace std;
using namespace Eigen;

void VisionTab::onButton(wxCommandEvent &evt) {

	// Check if a world exists
  if(mWorld == NULL) {
		printf("%c[%d;%dmCamera: Cannot handle event because a world is not loaded.%c[%dm\n",27,1,33,27,0);
		return;
	}

  // Traverse each robot in the world to check for a camera
	bool noCamera = true;
  for(int i = 0; i < mWorld->getNumRobots(); i++ ) { 
    robotics::Robot* robot = mWorld->getRobot(i);
    kinematics::BodyNode* cameraNode = robot->getNode("Camera");
    if(cameraNode != NULL) {
			noCamera = false;
			break;
		}
  }

	if(noCamera) {
    printf("%c[%d;%dmCamera: Cannot handle event because the world does not contain a camera.%c[%dm\n",27,1,33,27,0);
  	return;
	}
	
	// Get the button and switch on the set symbols
  int button_num = evt.GetId();
  switch (button_num) {
	  case button_attention:
			attention();
		break;
	  case button_startSearch:
			startSearch();
		break;
		case button_cloud:
			cloud();
		break;
		case button_depthMap:
			depthMap();
		break;	
	}
}

void VisionTab::attention () {

	// Set the predetermined configuration
	VectorXd conf (7);
	conf << 0.0, -0.648582, 0.0, 1.11803, 0.0, 1.62454, 0;
	mWorld->getRobot(0)->setQuickDofs(conf);
 	mWorld->getRobot(0)->update();
	robotics::Object* bear = mWorld->getObject(4);
	bear->setPositionXYZ(1.608, 2.4, 0.416);
	bear->update();
}

void VisionTab::startSearch () {

	// Read in the trajectory from the file
  ifstream in("../projects/tabVision/trajectory.txt");
	if(!in.is_open()) {
		printf("%c[%d;%dmCTabVision: Could not find the trajectory file.%c[%dm\n",27,1,33,27,0);
		return;
	}
	vector <VectorXd> trajectory;
	while(true) {
		VectorXd conf (7);
    for (size_t x = 0; x < 7; x++) 
      in >> conf(x);
		if(in.fail() || in.eof() || in.bad()) break; 
		trajectory.push_back(conf);
  }
	in.close();

	// Draw the trajectory
	size_t fps = 30;
	for(size_t i = 0; i < trajectory.size(); i++) {

		// Set the polar bear location
		double x, y, z;
		robotics::Object* bear = mWorld->getObject(4);
		bear->getPositionXYZ(x,y,z);
		if(i < 50) {}
		else if(i < 60)
			bear->setPositionXYZ(x,y-0.1,z);
		else if(i < 65)
			bear->setPositionXYZ(x-0.1,y-0.1,z);
		
		bear->update();

		// Set the robot location
		mWorld->getRobot(0)->setQuickDofs(trajectory[i]);
		mWorld->getRobot(0)->update();

		// Update the view
    usleep(1000000/fps);
    wxPaintEvent ev; 
	  viewer->render(ev);
	  camera->render(ev);
	}
}

void VisionTab::getDisparities (vector <Vector3d>& disparities, double& focalLength, Vector2d* range) {

	// Set the constants
	size_t kWidth = 640, kHeight = 480;
	float kBaseline = 0.10;
	float kDisparityNoise = 0.1;

	// Assert the baseline is non-negative
	assert((kBaseline > 0.0) && "Non-positive baseline.");

	printf("Starting to save... "); fflush(stdout);

	// Get the color data
	unsigned char* im = new unsigned char [3 * kWidth * kHeight];
	glReadPixels(0,0, kWidth, kHeight, GL_RGB, GL_UNSIGNED_BYTE, im);

	// Get the depths of the objects
	float* depths = new float [kWidth * kHeight];
	glReadPixels(0,0, kWidth, kHeight, GL_DEPTH_COMPONENT, GL_FLOAT, depths);

	// Get the view options
	glLoadIdentity();
	GLdouble modelMatrix[16];
	glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
	GLdouble projMatrix[16];
	glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
	int viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	// Get focal length using some pixel
	double x_, y_, z_;
	gluUnProject(300, 200, depths[200 * kWidth + 300], modelMatrix, projMatrix, viewport, &x_, &y_, &z_);
	focalLength = (300 - 320) * (-z_ / x_);

	// Create the pixel data and record the min and max disparities
	disparities.clear();
	double minDisp = 100000, maxDisp = 0;
	for(size_t v = 0; v < kHeight; v++) {
		for(size_t u = 0; u < kWidth; u++) {

			// Skip the pixel if it is in the background
			Vector2d pix (u,v); //= (*pix_it);
			size_t k = (pix(1) * kWidth + pix(0)) * 3;
			if((im[k] == 0) && (im[k+1] == 0) && (im[k+2] == 0)) continue;

			// Get the position data - if the 'z' is the far clip, skip
			size_t k2 = (pix(1) * kWidth + pix(0));
			double x_, y_, z_;
			gluUnProject(u, v, depths[k2], modelMatrix, projMatrix, viewport, &x_, &y_, &z_);
			Vector3d loc (x_, y_, z_);
			if(-loc(2) > 15.0) continue;		// TODO: Replace 15.0 with camera.zFar

			// Compute the disparity and add noise to it
			double disparity = (kBaseline * focalLength) / -loc(2);
		  float r1 = ((float) rand()) / RAND_MAX, r2 = ((float) rand()) / RAND_MAX;
		  float noiseEffect = sqrt(-2.0 * log(r1)) * cos(2 * M_PI * r2) * kDisparityNoise; 
			disparity += noiseEffect;

			// Update the min and max
			minDisp = min(minDisp, disparity);
			maxDisp = max(maxDisp, disparity);

			// Save the pixel location and the disparity
			size_t color = (im[k] << 16) | (im[k+1] << 8) | (im[k+2]);
			disparities.push_back(Vector3d(k2, disparity, color));
		}
	}

	// Set the min and max if necessary
	if(range != NULL) *range = Vector2d(minDisp, maxDisp);

	printf("Data acquired.\n"); fflush(stdout);
}

/// Prints the PCD file header
void VisionTab::printPCDHeader (FILE* file, size_t numPoints) {

  fprintf(file, "VERSION 0.7\n");
  fprintf(file, "FIELDS x y z rgb\n");
  fprintf(file, "SIZE 4 4 4 4\n");
  fprintf(file, "TYPE F F F U\n");
  fprintf(file, "COUNT 1 1 1 1\n");
  fprintf(file, "WIDTH 1\n");
  fprintf(file, "HEIGHT %lu\n", numPoints);
  fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
  fprintf(file, "POINTS %lu\n", numPoints);
  fprintf(file, "DATA ascii\n");
}

void VisionTab::cloud () {

	// Set the constants; TODO: Move these constants to class definition
	size_t kWidth = 640, kHeight = 480;
	float kBaseline = 0.10;

	// Get the disparities
	double focalLength;
	vector <Vector3d> disparities;
	getDisparities(disparities, focalLength);

	// Create the 3D points
	vector <Vector3d> points;
	for(size_t i = 0; i < disparities.size(); i++) {

		// Get the pixel
		size_t v = (size_t) disparities[i](0) / kWidth;
		size_t u = (size_t) disparities[i](0) % kWidth;

		// Change the location based on the new disparity
		double disparity = disparities[i](1);
		float distZ = -(kBaseline * focalLength) / disparity;
		float distX = ((int) u - (int) kWidth/2) * distZ / focalLength;
		float distY = ((int) v - (int) kHeight/2) * distZ / focalLength;

		// Save the pixel
		points.push_back(Vector3d(-distX, distY, -distZ));
	}

	// Save the points to a .pcd file
  FILE* file = fopen("cloud.pcd", "w");
  assert((file != NULL) && "The .pcd file could not be opened");

  // Write the pixel data to the file
  printPCDHeader(file, points.size());
	for(size_t i = 0; i < points.size(); i++) {
		Vector3d& loc = points[i];
    fprintf(file, "%lf\t%lf\t%lf\t%lu\n", loc(0), loc(1), loc(2), (size_t) disparities[i](2)); 
	}

  // Close the file
  assert((fclose(file) == 0) && "The file could not be closed successfully");

	// Run the pcd_viewer
	system("pcd_viewer cloud.pcd &");
  printf("PCD file saved.\n"); fflush(stdout);
}

void VisionTab::depthMap () {

	// Get the disparities
	double focalLength;
	vector <Vector3d> disparities;
	Vector2d minMax;
	getDisparities(disparities, focalLength, &minMax);
	double range = minMax(1) - minMax(0);

	// Create the wxImage
	const size_t kWidth = 640, kHeight = 480;
  unsigned char* imageData = (unsigned char*) malloc(kWidth * kHeight * 3); 
	memset(imageData, 9, kWidth * kHeight * 3);
	for(size_t i = 0; i < disparities.size(); i++) {

		// Get the pixel
		size_t v = (size_t) disparities[i](0) / kWidth;
		size_t u = (size_t) disparities[i](0) % kWidth;
		double disparity = disparities[i](1);

		// Set the pixel color
		size_t k = (u + v * kWidth) * 3;
		imageData[k] = imageData[k+1] = imageData[k+2] = (size_t) (((disparity - minMax(0)) / range) * 255);
	}

  wxImage img_ud(kWidth,kHeight,imageData);
  wxImage img = img_ud.Mirror(false);

	// Create the frame and visualize
	wxFrame* frame = new wxFrame(NULL, wxID_ANY, wxT("Hello wxDC"), wxPoint(50,50), wxSize(800,600));
 	wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
	wxImagePanel* drawPane = new wxImagePanel(frame, img);
  sizer->Add(drawPane, 1, wxEXPAND);
	frame->SetSizer(sizer);
	frame->Show();
}

VisionTab::VisionTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) : GRIPTab(parent, id, pos, size, style) {

	// ===========================================================
	// 1. Create the left side for the vision demonstration

	// Create StaticBox container for the two buttons: "Attention!" and "Start Search!"
	wxStaticBox* leftBox = new wxStaticBox(this, -1, wxT("Demonstration"));
	wxStaticBoxSizer* leftBoxSizer = new wxStaticBoxSizer(leftBox, wxVERTICAL);

	// Add the "Attention!" button
	leftBoxSizer->Add(new wxButton(this, button_attention, wxT("Attention!")), 0, wxALL, 10);

	// Add the "Start Search!" button
	leftBoxSizer->Add(new wxButton(this, button_startSearch, wxT("Find Bear!")), 0, wxALL, 10);

	// ===========================================================
	// 2. Create the right side for 3D data acquisition

	// Create StaticBox container for the two buttons: "Show Cloud" and "Show Depth Map"
	wxStaticBox* rightBox = new wxStaticBox(this, -1, wxT("3D Data"));
	wxStaticBoxSizer* rightBoxSizer = new wxStaticBoxSizer(rightBox, wxVERTICAL);

	// Add the "Attention!" button
	rightBoxSizer->Add(new wxButton(this, button_cloud, wxT("Show Cloud")), 0, wxALL, 10);

	// Add the "Go!" button
	rightBoxSizer->Add(new wxButton(this, button_depthMap, wxT("Show Depth Map")), 0, wxALL, 10);

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

// Handles for the ImagePanel
BEGIN_EVENT_TABLE(wxImagePanel, wxPanel)
EVT_PAINT(wxImagePanel::paintEvent)
END_EVENT_TABLE()

// Add a handlers for UI changes
BEGIN_EVENT_TABLE(VisionTab, wxPanel)
	EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, VisionTab::onButton)
END_EVENT_TABLE ()

// Class constructor for the tab
IMPLEMENT_DYNAMIC_CLASS(VisionTab, GRIPTab)


