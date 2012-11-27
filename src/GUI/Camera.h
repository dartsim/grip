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

#pragma once

#include <Eigen/Core>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <kinematics/Shape.h>
#include <kinematics/BodyNode.h>
#include <kinematics/ShapeMesh.h>
#include <renderer/OpenGLRenderInterface.h>
#include <Tools/Constants.h>

using namespace Eigen;

class Camera: public wxGLCanvas {
private:

	// Changed with the resized function (mainly)
	int width;			///< The width of the canvas
	int height;			///< The height of the canvas

	renderer::OpenGLRenderInterface renderer;			///< The OpenGL renderer interface

public:
	kinematics::BodyNode* cameraNode; 						///< The body node of the camera
	bool sceneChanged;

public:

	/// The constructor
	Camera( wxWindow * parent, wxWindowID id, const wxPoint & pos,
		const wxSize& size, long style = 0, const wxString & name =
					_("GLCanvas"), int * attribList = 0,
		const wxPalette & palette = wxNullPalette) :
		wxGLCanvas(parent, id, pos, size, style, name, attribList, palette),		// Changed for wx 2.8
	//	wxGLCanvas(parent, id, attribList, pos, size, style, name, palette),
		cameraNode(NULL)
	{
		sceneChanged = true;
	}

	/// The destructor
	virtual ~Camera() {
		renderer.destroy();
	}

	void InitGL();						///< Initializes OpenGL options and sets the class variables
	int  DrawGLScene();				///< If the world contains a robot with a camera, draws the world from the camera's perspective

	// TODO Find a way not to duplicate code between these implementations and those in Viewer.cpp
	void drawWorld(); 				///< Draws the world
	void drawModel(const aiScene* _model, Eigen::Transform<double, 3, Eigen::Affine> *_pose, bool collisionFlag);	///< Draws the object models

	void resized(wxSizeEvent& evt);			///< Resizes the canvas 
	void shown(wxShowEvent& evt);				///< Draws the canvas for wxWidget "show" event
	void render(wxPaintEvent& evt);			///< Main interface for rendering - calls DrawOpenGL

// Functions needed to wxWidget
public:

	/// Waits for events
	void OnIdle(wxIdleEvent & evt) {
 		wxPaintEvent ev;
	//	if(sceneChanged) 
		render(ev);

//		Refresh(false);
		evt.RequestMore();
	}

	/// Return the width of the canvas
	int getWidth() {
		return GetSize().x;
	}
	
	/// Return the height of the canvas
	int getHeight() {
		return GetSize().y;
	}

	/// Declare the events the canvas should respond to
	DECLARE_EVENT_TABLE()
};
