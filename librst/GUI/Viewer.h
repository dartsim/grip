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

#ifndef VIEWER_H
#define VIEWER_H

//#define EIGEN_DONT_ALIGN
// TODO Fix Eigen Alignment issues: http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <Tools/GL/glcommon.h>
#include <Tools/Model3DS.h>

using namespace Eigen;

class Viewer: public wxGLCanvas {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Transform<double, 3, Eigen::Affine> camT, prevCamT;
	Matrix3d camRotT;
	Vector3d worldV, prevWorldV;

	Model3DS* model;

	Viewer(wxWindow * parent, wxWindowID id, const wxPoint & pos,
			const wxSize& size, long style = 0, const wxString & name =
					_("GLCanvas"), int * attribList = 0,
			const wxPalette & palette = wxNullPalette) :
		wxGLCanvas(parent, id, pos, size, style, name, attribList, palette) {
	}

	virtual ~Viewer() {
	}


	void OnIdle(wxIdleEvent & evt) {
		//draw();
		evt.RequestMore();
	}

	void InitGL();
	void setClearColor();
	void UpdateCamera();
	int  DrawGLScene();
	void ResetGL();
	void addGrid();

	long x, y, xInit, yInit;
	int w, h;

	bool existsUpdate;
	bool doCollisions;
	bool Move;
	bool pflag;
	int threadCounter;
	bool gridActive;
	double camRadius;
	bool keys[256];
	bool active;
	bool renderLoopActive;
	bool redrawFlag;
	bool showSpheres;
	bool fullscreen;
	///////////////////////
	bool mouseLDown;
	bool mouseRDown;
	bool mouseMDown;
	int redrawCount;
	bool loading;

	Vector3d gridColor;
	Vector3d backColor;

	bool mouseCaptured;

	void resized(wxSizeEvent& evt);
	void shown(wxShowEvent& evt);

	int getWidth() {
		return GetSize().x;
	}
	int getHeight() {
		return GetSize().y;
	}

	void render(wxPaintEvent& evt);

	// events
	void OnMouse(wxMouseEvent& evt);
	void OnCaptureLost(wxMouseCaptureLostEvent& evt);
	void mouseWheelMoved(wxMouseEvent& evt);

DECLARE_EVENT_TABLE()
};

#endif
