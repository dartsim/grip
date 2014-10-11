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

#include "Viewer.h"
#include "wx/sizer.h"
#include "GUI.h"
#include "GRIPFrame.h"
#include <wx/glcanvas.h>

#include <Tools/GL/glcommon.h>
#include <Tools/Constants.h>
#include <iostream>

using namespace std;

/**
 * @function drawWorld
 * @brief Draw World and everything inside it: Robots + Objects
 */
void Viewer::drawWorld() {
	// Draw skeletons
//	cout << "==========================" << endl;
//	renderer.draw(mWorld->getSkeleton("PointInput2"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("PointLeverFulcrum"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Intersect1"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Load"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Fulcrum"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Wall3"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Wall3b"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("PointLeverLoad"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Krang"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("ground"), check_for_collisions, useCollMesh);
//	renderer.draw(mWorld->getSkeleton("Lever"), check_for_collisions, useCollMesh);

	for(int i=0; i < mWorld->getNumSkeletons(); i++) {
		//mWorld->getSkeleton(i)->draw(&renderer);
		renderer.draw(mWorld->getSkeleton(i), check_for_collisions, useCollMesh);
	}
}

/**
 * @function shown
 */
void Viewer::shown(wxShowEvent& WXUNUSED(evt)){
    int w, h;
    GetClientSize(&w, &h);
	#ifndef __WXMOTIF__
    if (GetContext())
	#endif
    {
        SetCurrent();
        glViewport(0, 0, (GLint) w, (GLint) h);
    }
	DrawGLScene();
}

/**
 * @function UpdateCamera
 */
void Viewer::UpdateCamera(void){
	camT = camRotT;
	camT.translate(Vector3d(camRadius, 0, 0));
}

/**
 * @function DrawGLScene
 */
int Viewer::DrawGLScene()
{
	glPolygonMode (GL_FRONT, GL_FILL);
	glEnable(GL_BLEND);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glHint(GL_FOG_HINT,GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

	gluLookAt(camT(0,3), camT(1,3), camT(2,3),   // Camera position
              0, 0, 0,                           // Target Vector in scene
              camT(0,2), camT(1,2), camT(2,2));  // Up Vector from Camera pose

	glEnable(GL_TEXTURE_2D);
	glColor4f(1.0f, 1.0f, 1.0f, 0.0f);

	// Lighting settings
	GLfloat position[]= {static_cast<GLfloat>(camT(0,3)),
						 static_cast<GLfloat>(camT(1,3)),
						 static_cast<GLfloat>(camT(2,3)),
						 1.0}; // Camera position
	//	GLfloat position[] = {1.0,0.0,0.0,0.0};
	GLfloat position1[] = {-1.0,0.0,0.0,0.0};

	static float ambient[]             = {0.2, 0.2, 0.2, 1.0};
	static float diffuse[]             = {0.6, 0.6, 0.6, 1.0};
	static float front_mat_shininess[] = {60.0};
	static float front_mat_specular[]  = {0.2, 0.2,  0.2,  1.0};
	static float front_mat_diffuse[]   = {0.5, 0.28, 0.38, 1.0};
	static float lmodel_ambient[]      = {0.2, 0.2,  0.2,  1.0};
	static float lmodel_twoside[]      = {GL_FALSE};

	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT,  ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE,  diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,  lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	// Back light
	//	glEnable( GL_LIGHT1);
	//	glLightfv(GL_LIGHT1,GL_DIFFUSE, diffuse);
	//	glLightfv(GL_LIGHT1,GL_POSITION, position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, front_mat_shininess);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  front_mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   front_mat_diffuse);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	// Render
	glTranslated(worldV[0],worldV[1],worldV[2]);

	glPushMatrix();
	if (gridActive){  addGrid(); }
	glPopMatrix();

	//USUALLY BAD
	glDisable(GL_FOG);

        // draw models
	if( mWorld != NULL ) { 
		drawWorld();
	}

	// fire during-render hooks
	((GRIPFrame*)GetParent())->FireEventRender();

	glFlush();
	SwapBuffers();
	return TRUE;
}

/**
 * @function resized
 * @brief 
 */
void Viewer::resized(wxSizeEvent& evt){
	if(!handleEvents) return;

	if(!IsShown() || !GetParent()->IsShown()) return;
	wxGLCanvas::OnSize(evt);
    int w, h;
    GetClientSize(&w, &h);
	#ifndef __WXMOTIF__
    if (GetContext())
	#endif
    {
        SetCurrent();
        glViewport(0, 0, (GLint) w, (GLint) h);
    }
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f,(GLdouble)w/(GLdouble)h,0.1f,15.0f);//100.0f);
	DrawGLScene();
}

/**
 * @function mouseWheelMoved
 * @brief Update the radius of the camera (rotation) according to the mouse's wheel
 */
void Viewer::mouseWheelMoved(wxMouseEvent& evt){
	if(!handleEvents) return;
	SetFocus();
	int wheelRot = evt.GetWheelRotation();

	double camR = camRadius - (double)wheelRot / 500.f;
	if(camR > .05){
		camRadius = camR;
		UpdateCamera();
		DrawGLScene();
	}
}

/**
 * @function OnCaptureLost
 */
void Viewer::OnCaptureLost(wxMouseCaptureLostEvent& WXUNUSED(evt)){
	mouseCaptured = false;
}

/**
 * @function OnMouse
 */
void Viewer::OnMouse(wxMouseEvent& evt){
	if(!handleEvents) return;
	evt.GetPosition(&x,&y);

	if(evt.ButtonUp() && mouseCaptured){
		ReleaseMouse();
		mouseCaptured = false;
	}

	if(evt.ButtonDown() && !mouseCaptured){
		prevCamT = camT;
		prevWorldV = worldV;
		xInit = x;
		yInit = y;
		SetFocus();
		CaptureMouse();
		mouseCaptured = true;
		return;
	}
	double dx = (double)(x-xInit);
	double dy = (double)(y-yInit);

	if (evt.LeftIsDown() && evt.RightIsDown()){
		xInit = x;
		yInit = y;
		double camR = camRadius + dy *CAMERASPEED* 2.0f;
		if(camR > .05)
			camRadius = camR;
		UpdateCamera();
		DrawGLScene();
	}else if(evt.LeftIsDown()){
		double theta,phi;
		theta = -(dx/90.f);
		phi = -(dy/90.f);

		Matrix3d tm,pm;
		tm = AngleAxisd(theta, Vector3d::UnitZ());
		pm = AngleAxisd(phi, Vector3d::UnitY());

		Matrix3d drot = prevCamT.rotation();
		drot *= tm;
		drot *= pm;
		camRotT = drot;

		UpdateCamera();
		DrawGLScene();
	}else if(evt.MiddleIsDown() || evt.RightIsDown()){

		Vector3d dispV = camRotT*Vector3d(0,(double)dx * CAMERASPEED , -(double)dy * CAMERASPEED);
		worldV = prevWorldV+dispV;
		UpdateCamera();
		DrawGLScene();
	}
}

/**
 * @function render
 * @brief Render image
 */
void Viewer::render(wxPaintEvent& WXUNUSED(evt)){
    if(!IsShown()) return;
    wxGLCanvas::SetCurrent();
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	DrawGLScene();
}

/**
 * @function InitGL
 * @brief Init 
 */
void Viewer::InitGL(){
	doCollisions=false;
	gridActive = true;
	mouseCaptured = false;

	xInit=0; yInit=0; x=0; y=0;

    GetClientSize(&w, &h);
	#ifndef __WXMOTIF__
    if (GetContext())
	#endif
    {
		Show();
		SetCurrent();
        glViewport(0, 0, (GLint) w, (GLint) h);
    }

	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(100.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	float FogCol[3]={0.0f,0.0f,0.0f};
	glFogfv(GL_FOG_COLOR,FogCol);
	glFogf(GL_FOG_START, 10.f);
	glFogf(GL_FOG_END, 15.f);
	glFogi(GL_FOG_MODE, GL_LINEAR);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glPolygonMode (GL_FRONT, GL_FILL);
	gluPerspective(45.0f,(GLdouble)w/(GLdouble)h,0.1f,15.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	setClearColor();
}

/**
 * @function setClearColor
 * @brief No idea
 */
void Viewer::setClearColor(){
	glClearColor((float)backColor[0],(float)backColor[1],(float)backColor[2],1.0f);
	float FogCol[3]={(float)backColor[0],(float)backColor[1],(float)backColor[2]};
	glFogfv(GL_FOG_COLOR,FogCol);
}

/**
 * @function addGrid
 * @brief Add a grid in the scene
 */
void Viewer::addGrid(){
	glEnable(GL_FOG);
	double sizeX=100.0f;
	double sizeY=100.0f;

	double startX=-sizeX/2.f;
	double endX  =sizeX/2.f;
	double startY=-sizeY/2.f;
	double endY  =sizeY/2.f;

	GLdouble grid2x2[2][2][3] = {
		{{startX, startY, 0.0}, {endX, startY, 0.0}},
		{{startX, endY, 0.0}, {endX, endY, 0.0}}
	};

	GLdouble *grid = &grid2x2[0][0][0];

    glEnable(GL_MAP2_VERTEX_3);
    glMap2d(GL_MAP2_VERTEX_3,
    0.0, 1.0,  /* U ranges 0..1 */
    3,         /* U stride, 3 floats per coord */
    2,         /* U is 2nd order, ie. linear */
    0.0, 1.0,  /* V ranges 0..1 */
    2 * 3,     /* V stride, row is 2 coords, 3 floats per coord */
    2,         /* V is 2nd order, ie linear */
    grid);  /* control points */

	glMapGrid2f(
    100, 0.0, 1.0,
    100, 0.0, 1.0);


	glLineWidth(0.9f);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);

	//glColor3f(.2f,.2f,0.0f);
	glColor3d(gridColor[0],gridColor[1],gridColor[2]);

    glEvalMesh2(GL_LINE,
    0, 100,   /* Starting at 0 mesh 100 steps (rows). */
    0, 100);  /* Starting at 0 mesh 100 steps (columns). */

	glColor3f(1.0f,1.0f,1.0f);
	glEnable(GL_LIGHTING);
	glDisable(GL_FOG);
	glDisable(GL_COLOR_MATERIAL);
}


BEGIN_EVENT_TABLE(Viewer, wxGLCanvas)
	EVT_SHOW(Viewer::shown)
    EVT_SIZE(Viewer::resized)
    EVT_PAINT(Viewer::render)
	EVT_MOUSE_CAPTURE_LOST(Viewer::OnCaptureLost)
	EVT_MOUSEWHEEL(Viewer::mouseWheelMoved)
    EVT_MOTION(Viewer::OnMouse)
	EVT_RIGHT_DOWN(Viewer::OnMouse)
	EVT_MIDDLE_DOWN(Viewer::OnMouse)
	EVT_LEFT_DOWN(Viewer::OnMouse)
	EVT_LEFT_UP(Viewer::OnMouse)
	EVT_RIGHT_UP(Viewer::OnMouse)
	EVT_MIDDLE_UP(Viewer::OnMouse)
END_EVENT_TABLE()

