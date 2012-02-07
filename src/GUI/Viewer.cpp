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
#include <wx/glcanvas.h>

#include <Tools/GL/glcommon.h>
#include <Tools/Constants.h>
#include <Tools/Collision.h>

#include <iostream>

using namespace std;

extern bool check_for_collisions;
/**
 * @function drawWorld
 * @brief Draw World and everything inside it: Robots + Objects
 */

void Viewer::drawConfig(config_t _c) {
	pattern_ pattern = _c.get_pattern();
	key_ key = _c.get_key();
	dimensions_ dims = _c.get_dimensions();

	glPushMatrix();
	glColor3f(0.3f, 0.3f, 0.3f);
	float b_w = d.bin.w / keyScale;
	float b_h = d.bin.h / keyScale;
	float b_d = d.bin.d / keyScale;
	glTranslatef(b_w / 2.0, b_h / 2.0, -0.05);
	glScalef(b_w, b_h, 0.1);
	glutSolidCube(1.0f);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0.3f, 0.3f, 0.3f);
	glTranslatef(b_w / 2.0, b_h / 2.0, b_d / 2.0);
	glScalef(b_w, b_h, b_d);
	glutWireCube(1.0f);
	glPopMatrix();

	glPushMatrix();
	b_w = dims[0] / keyScale;
	b_h = dims[1] / keyScale;
	b_d = dims[2] / keyScale;
	glColor3f(0.7f, 0.7f, 0.7f);
	glTranslatef(b_w / 2.0, b_h / 2.0, b_d / 2.0);
	glScalef(b_w+0.01, b_h+0.01, b_d+0.01);
	glutWireCube(1.0f);
	glPopMatrix();

	int c = 0;
	for (uint i = 0; i < key.size(); i++) {
		for (int j = 0; j < key[i]; j++) {
			glPushMatrix();
			float w = d.package[i].w / keyScale;
			float h = d.package[i].h / keyScale;
			float _d = d.package[i].d / keyScale;
			float pos_x = pattern[c * 3 + 0] / keyScale;
			float pos_y = pattern[c * 3 + 1] / keyScale;
			float pos_z = pattern[c * 3 + 2] / keyScale;
			pos_x += w / 2.0;
			pos_y += h / 2.0;
			pos_z += _d / 2.0;
			glTranslatef(pos_x, pos_y, pos_z);
			glScalef(w, h, _d);
			glColor3f(c_red[i], c_green[i], c_blue[i]);
			glutSolidCube(1.0f);
			glColor3f(0.0f, 0.0f, 0.0f);
			glutWireCube(1.0f);
			glPopMatrix();
			c++;
		}
	}

}

void Viewer::drawPacklist(vector<config_t> _pl) {
	for (uint i = 0; i < _pl.size(); i++) {
		drawConfig(_pl[i]);
	}
}

void Viewer::drawWorld() {

	if (check_for_collisions) {
		mCollision->UpdateAllCollisionModels();
	}

	// Draw Objects
	for (unsigned int i = 0; i < mWorld->mObjects.size(); i++) {

		for (int j = 0; j < mWorld->mObjects[i]->getNumNodes(); j++) {

			Eigen::Matrix4d poseMatrix =
					mWorld->mObjects[i]->getNode(j)->getWorldTransform();
			Transform<double, 3, Affine> pose;
			pose.setIdentity();
			pose.matrix() = poseMatrix;
			drawModel(mWorld->mObjects[i]->mModels[0], &pose);
		}
	}

	// Draw Robot
	for (unsigned int i = 0; i < mWorld->mRobots.size(); i++) {

		for (unsigned int j = 0; j < mWorld->mRobots[i]->mModels.size(); j++) {

			int ind = mWorld->mRobots[i]->mModelIndices[j];
			Eigen::Matrix4d poseMatrix =
					mWorld->mRobots[i]->getNode(ind)->getWorldTransform();

			Transform<double, 3, Affine> pose;
			pose.matrix() = poseMatrix;
			drawModel(mWorld->mRobots[i]->mModels[j], &pose);
		}
	}

}

/**
 * @function drawModel
 */
void Viewer::drawModel(Model3DS* _model, Transform<double, 3, Affine> *_pose) {
	if (_model == NULL)
		return;

	if (check_for_collisions && _model->collisionFlag) {
		glDisable(GL_TEXTURE_2D);
		glColor3f(1.0f, .1f, .1f);
	}

	glPushMatrix();
	glMultMatrixd(_pose->data());
	/*
	 if(comFlag){
	 glPushMatrix();
	 glColor3f(0.0f, 1.0f, 0.0f);
	 glTranslated(COM(0),COM(1),COM(2));
	 DrawSphere(0.02f, 10, 10);
	 glPopMatrix();

	 glEnable(GL_POLYGON_STIPPLE);
	 glPolygonStipple(halftone);
	 glDisable(GL_TEXTURE_2D);
	 glColor3d(0.20f,0.20f,0.20f);
	 }
	 */
	if (_model != NULL) {
		glCallList(_model->modelDL);
		//glCallList(model->colDL);  // For testing collision model
	}

	glDisable(GL_POLYGON_STIPPLE);

	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_TEXTURE_2D);
	glPopMatrix();

}

/**
 * @function shown
 */
void Viewer::shown(wxShowEvent& WXUNUSED(evt)) {
	int w, h;
	GetClientSize(&w, &h);
#ifndef __WXMOTIF__
	if (GetContext())
#endif
	{
		SetCurrent();
		glViewport(0, 0, (GLint) w, (GLint) h);
	}
	UpdateCamera();
}

/**
 * @function UpdateCamera
 */
void Viewer::UpdateCamera(void) {
	SetCurrent();
	camT = camRotT;
	camT.translate(Vector3d(camRadius, 0, 0));
	DrawGLScene();
}

/**
 * @function DrawGLScene
 */int Viewer::DrawGLScene() {
	redrawFlag = true;
	glLoadIdentity();
	glPolygonMode(GL_FRONT, GL_FILL);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glHint(GL_FOG_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	gluLookAt(camT(0, 3), camT(1, 3), camT(2, 3), // Camera position
			0, 0, 0, // Target Vector in scene
			camT(0, 2), camT(1, 2), camT(2, 2)); // Up Vector from Camera pose

	float position[] = { camT(0, 3), camT(1, 3), camT(2, 3), 100.0 }; // Camera position
	glEnable(GL_TEXTURE_2D);
	glColor4f(1.0f, 1.0f, 1.0f, 0.0f);

	glPushMatrix();

	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glPushMatrix();

	float no_mat[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float diffuse[] = { .7f, .7f, .7f, .7f };
	float specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	glLightfv(GL_LIGHT1, GL_POSITION, position);
	glLightfv(GL_LIGHT1, GL_AMBIENT, no_mat);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);

	GLfloat HeadlightAmb[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat HeadlightDif[4] = { 1.0f, 1.0f, 1.0f, 0.0f };
	GLfloat HeadlightSpc[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	glLightfv(GL_LIGHT2, GL_AMBIENT, HeadlightAmb);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, HeadlightDif);
	glLightfv(GL_LIGHT2, GL_SPECULAR, HeadlightSpc);
	glLightfv(GL_LIGHT2, GL_POSITION, position);

	glPopMatrix();

	glTranslated(worldV[0], worldV[1], worldV[2]);

	float ambRefl[] = { 0.8f, 0.8f, 0.8f, 1.0f }; // default
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambRefl);
	float diffRefl[] = { 0.8f, 0.8f, 0.8f, 1.0f }; // default
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffRefl);
	float specRefl[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // default
	glMaterialfv(GL_FRONT, GL_SPECULAR, specRefl);

	glPushMatrix();
	if (gridActive) {
		addGrid();
	}
	glPopMatrix();

	//USUALLY BAD
	glDisable(GL_FOG);

	glPushMatrix();
	if (mWorld != NULL) {
		drawWorld();
	}

	if (keyChanged == 1) drawConfig(configCurrent);
	else if (keyChanged == 2) drawPacklist(packlistCurrent);

	glPopMatrix();

	glPopMatrix();

	glFlush();
	SwapBuffers();
	return TRUE;
}

/**
 * @function resized
 * @brief 
 */
void Viewer::resized(wxSizeEvent& evt) {
	if (!IsShown() || !GetParent()->IsShown())
		return;
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
	glPolygonMode(GL_FRONT, GL_FILL);
	gluPerspective(45.0f, (GLdouble) w / (GLdouble) h, 0.1f, 1000.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	DrawGLScene();
}

/**
 * @function mouseWheelMoved
 * @brief Update the radius of the camera (rotation) according to the mouse's wheel
 */
void Viewer::mouseWheelMoved(wxMouseEvent& evt) {
	SetFocus();
	int wheelRot = evt.GetWheelRotation();

	double camR = camRadius - (double) wheelRot / 500.f;
	if (camR > .05) {
		camRadius = camR;
		UpdateCamera();
	}
}

/**
 * @function OnCaptureLost
 */
void Viewer::OnCaptureLost(wxMouseCaptureLostEvent& WXUNUSED(evt)) {
	mouseCaptured = false;
}

/**
 * @function OnMouse
 */
void Viewer::OnMouse(wxMouseEvent& evt) {
	evt.GetPosition(&x, &y);

	if (evt.ButtonUp() && mouseCaptured) {
		ReleaseMouse();
		mouseCaptured = false;
	}

	if (evt.ButtonDown() && !mouseCaptured) {
		prevCamT = camT;
		prevWorldV = worldV;
		xInit = x;
		yInit = y;
		SetFocus();
		CaptureMouse();
		mouseCaptured = true;
		return;
	}
	double dx = (double) (x - xInit);
	double dy = (double) (y - yInit);

	if (evt.LeftIsDown() && evt.RightIsDown()) {
		xInit = x;
		yInit = y;
		double camR = camRadius + dy * CAMERASPEED * 2.0f;
		if (camR > .05)
			camRadius = camR;
		UpdateCamera();
	} else if (evt.LeftIsDown()) {
		double theta, phi;
		theta = -(dx / 90.f);
		phi = -(dy / 90.f);

		Matrix3d tm, pm;
		tm = AngleAxisd(theta, Vector3d::UnitZ());
		pm = AngleAxisd(phi, Vector3d::UnitY());

		Matrix3d drot = prevCamT.rotation();
		drot *= tm;
		drot *= pm;
		camRotT = drot;

		existsUpdate = true;
		UpdateCamera();

	} else if (evt.MiddleIsDown() || evt.RightIsDown()) {

		Vector3d dispV = camRotT * Vector3d(0, (double) dx * CAMERASPEED,
				-(double) dy * CAMERASPEED);
		worldV = prevWorldV + dispV;
		existsUpdate = false;
		UpdateCamera();
	}

	if (loading) {
		loading = false;
		ResetGL();
	}
}

/**
 * @function render
 * @brief Render image
 */
void Viewer::render(wxPaintEvent& WXUNUSED(evt)) {
	if (!IsShown())
		return;
	wxGLCanvas::SetCurrent();
	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	DrawGLScene();
}

/**
 * @function InitGL
 * @brief Init 
 */
void Viewer::InitGL() {
	loading = false;
	existsUpdate = false;
	doCollisions = false;
	Move = false;
	pflag = false;
	threadCounter = 0;
	gridActive = true;
	redrawFlag = true;
	gridActive = true;
	mouseLDown = false;
	mouseRDown = false;
	mouseMDown = false;
	mouseCaptured = false;

	xInit = 0;
	yInit = 0;
	x = 0;
	y = 0;

	redrawCount = 0;

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
	glClearDepth(200.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glDepthFunc(GL_LEQUAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	float FogCol[3] = { 0.0f, 0.0f, 0.0f };
	glFogfv(GL_FOG_COLOR, FogCol);
	glFogf(GL_FOG_START, 10.f);
	glFogf(GL_FOG_END, 15.f);
	glFogi(GL_FOG_MODE, GL_LINEAR);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glPolygonMode(GL_FRONT, GL_FILL);
	gluPerspective(45.0f, (GLdouble) w / (GLdouble) h, 0.1f, 150.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	UpdateCamera();

	backColor = Vector3d(0, 0, 0);
	gridColor = Vector3d(.5, .5, .0);

	camRotT = AngleAxis<double> (DEG2RAD(-30), Vector3d(0, 1, 0));
	prevCamT = camRotT;
	worldV = Vector3d(0, 0, 0);
	prevWorldV = worldV;
	camRadius = defaultCamRadius;
}

/**
 * @function ResetGL
 * @brief Reset the Viewer
 */
void Viewer::ResetGL() {

	GetClientSize(&w, &h);
	SetCurrent();
	glViewport(0, 0, (GLint) w, (GLint) h);

	glFlush();
	glFinish();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glPolygonMode(GL_FRONT, GL_FILL);
	gluPerspective(45.0f, (GLdouble) w / (GLdouble) h, 0.1f, 150.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	setClearColor();
	UpdateCamera();
}

/**
 * @function setClearColor
 * @brief No idea
 */
void Viewer::setClearColor() {
	glClearColor((float) backColor[0], (float) backColor[1],
			(float) backColor[2], 1.0f);
	float FogCol[3] = { (float) backColor[0], (float) backColor[1],
			(float) backColor[2] };
	glFogfv(GL_FOG_COLOR, FogCol);
}

/**
 * @function addGrid
 * @brief Add a grid in the scene
 */
void Viewer::addGrid() {
	glEnable(GL_FOG);
	double sizeX = 100.0f;
	double sizeY = 100.0f;

	double startX = -sizeX / 2.f;
	double endX = sizeX / 2.f;
	double startY = -sizeY / 2.f;
	double endY = sizeY / 2.f;

	GLfloat grid2x2[2][2][3] = { { { startX, startY, 0.0 },
			{ endX, startY, 0.0 } }, { { startX, endY, 0.0 },
			{ endX, endY, 0.0 } } };

	GLfloat *grid = &grid2x2[0][0][0];

	glEnable(GL_MAP2_VERTEX_3);
	glMap2f(GL_MAP2_VERTEX_3, 0.0, 1.0, /* U ranges 0..1 */
	3, /* U stride, 3 floats per coord */
	2, /* U is 2nd order, ie. linear */
	0.0, 1.0, /* V ranges 0..1 */
	2 * 3, /* V stride, row is 2 coords, 3 floats per coord */
	2, /* V is 2nd order, ie linear */
	grid); /* control points */

	glMapGrid2f(100, 0.0, 1.0, 100, 0.0, 1.0);

	glLineWidth(0.9f);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);

	//glColor3f(.2f,.2f,0.0f);
	glColor3d(gridColor[0], gridColor[1], gridColor[2]);

	glEvalMesh2(GL_LINE, 0, 100, /* Starting at 0 mesh 100 steps (rows). */
	0, 100); /* Starting at 0 mesh 100 steps (columns). */

	glColor3f(1.0f, 1.0f, 1.0f);
	glEnable(GL_LIGHTING);
	glDisable(GL_FOG);
}
BEGIN_EVENT_TABLE(Viewer, wxGLCanvas)
EVT_SHOW(Viewer::shown) EVT_SIZE(Viewer::resized)
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

