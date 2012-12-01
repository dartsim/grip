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

/**
 * @file tabVision.h
 * @author Can Erdogan
 * @date Nov 27, 2012
 * @brief The tab description which will use the "SchunkArmKinect.urdf"
 * to demonstrate the vision user interface and the ability to get
 * noisy depth information using OpenGL.
 */

#pragma once

#include <Tabs/GRIPTab.h>
#include <Tools/Constants.h>

/// The definition of a new frame to show an image
class wxImagePanel : public wxPanel {
public:
  wxBitmap image;			///< The shown image
	/// The constructor
  wxImagePanel(wxFrame* parent, const wxImage& img) : wxPanel(parent) {
    image = wxBitmap(img);
  }
	/// The paint callback
  void paintEvent(wxPaintEvent & evt) {
    wxPaintDC dc(this);
    dc.DrawBitmap( image, 0, 0, false );
  }
	/// Handlers table for wxWidget
  DECLARE_EVENT_TABLE()
};

/// The tab definition
class VisionTab : public GRIPTab
{
public:

	/// The callback events
	enum Events {
		button_attention,
		button_startSearch,
		button_cloud,
		button_depthMap
	};

	/// The default constructor
	VisionTab(){};

	/// The constructor with the parent window and etc. 
	VisionTab(wxWindow * parent, wxWindowID id = wxID_ANY,
			 const wxPoint & pos = wxDefaultPosition,
			 const wxSize & size = wxDefaultSize,
			 long style = wxTAB_TRAVERSAL);

	/// The destructor
	virtual ~VisionTab(){}

	/// Moves the arm to a position where the scene is probably visible through the camera
	void attention ();

	/// Moves the arm around to visualize more of the scene
	void startSearch ();

	/// Prints a pcd header to the file
	void printPCDHeader (FILE* file, size_t numPoints);

	/// Returns the disparity, color and pixel location for the visible 3D points
	void getDisparities (std::vector <Eigen::Vector3d>& disparities, double& focalLength, Eigen::Vector2d* range = NULL);

	/// Displays the point cloud in a new window after getting the depth values
	void cloud ();

	/// Displays the depth map mapped to a greyscale image
	void depthMap ();

	/// Handles the state changes in GRIP
	void GRIPStateChange(){}

	/// Handles the input from button interfaces
	void onButton(wxCommandEvent &evt);

// The macros for wxWidget
public:
	DECLARE_DYNAMIC_CLASS(VisionTab)
	DECLARE_EVENT_TABLE()
};
