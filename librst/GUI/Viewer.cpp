//---------------------------------------------------------------------
//  Copyright (c) 2008 Mike Stilman
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

#include "Viewer.h"
#include "../Tools/World.h"
#include "wx/sizer.h"
#include "GUI.h"
#include <wx/glcanvas.h>

#include <Tools/Robot.h>
#include <Tools/Link.h>
#include <iostream>

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
	UpdateCamera();
}

void Viewer::UpdateCamera(void){
	SetCurrent();
	camT = camRotT;
	camT.translate(Vector3d(camRadius, 0, 0));
	DrawGLScene();
}

int Viewer::DrawGLScene()
{
	redrawFlag = true;
	glLoadIdentity();
	glPolygonMode (GL_FRONT, GL_FILL);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glHint(GL_FOG_HINT,GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

	gluLookAt(camT(0,3),camT(1,3),camT(2,3), // Camera position
			  0,0,0, // Target Vector in scene
			  camT(0,2),camT(1,2),camT(2,2));  // Up Vector from Camera pose

	float position[]= {camT(0,3),camT(1,3),camT(2,3), 1.0}; // Camera position
	glEnable(GL_TEXTURE_2D);
	glColor4f(1.0f,1.0f,1.0f,0.0f);

	glPushMatrix();

	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glPushMatrix();

	float no_mat[] = {0.0f, 0.0f, 0.0f, 1.0f};
	float diffuse[] = {.7f, .7f, .7f, .7f};
	float specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
	glLightfv(GL_LIGHT1, GL_POSITION, position);
	glLightfv(GL_LIGHT1, GL_AMBIENT, no_mat);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);

	GLfloat HeadlightAmb[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat HeadlightDif[4] = {1.0f, 1.0f, 1.0f, 0.0f};
	GLfloat HeadlightSpc[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	glLightfv(GL_LIGHT2, GL_AMBIENT, HeadlightAmb);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, HeadlightDif);
	glLightfv(GL_LIGHT2, GL_SPECULAR, HeadlightSpc);
	glLightfv(GL_LIGHT2, GL_POSITION, position);


	glPopMatrix();

	glTranslated(worldV[0],worldV[1],worldV[2]);

	float ambRefl[] = {0.2f, 0.2f, 0.2f, 1.0f}; // default
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambRefl);
	float diffRefl[] = {0.8f, 0.8f, 0.8f, 1.0f}; // default
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffRefl);
	float specRefl[] = {1.0f, 1.0f, 1.0f, 1.0f}; // default
	glMaterialfv(GL_FRONT, GL_SPECULAR, specRefl);

	glPushMatrix();
	if (gridActive){  addGrid(); }
	glPopMatrix();

	//USUALLY BAD
	glDisable(GL_FOG);

	glPushMatrix();
	if(world!=NULL) world->Draw();
	glPopMatrix();

	glPopMatrix();

	glFlush();
	SwapBuffers();
	return TRUE;
}


void Viewer::resized(wxSizeEvent& evt){
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
	glPolygonMode (GL_FRONT, GL_FILL);
	gluPerspective(45.0f,(GLdouble)w/(GLdouble)h,0.1f,100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	DrawGLScene();
}

void Viewer::mouseWheelMoved(wxMouseEvent& evt){
	SetFocus();
	int wheelRot = evt.GetWheelRotation();

	double camR = camRadius - (double)wheelRot / 500.f;
	if(camR > .05){
		camRadius = camR;
		UpdateCamera();
	}
}

void Viewer::OnCaptureLost(wxMouseCaptureLostEvent& WXUNUSED(evt)){
	mouseCaptured = false;
}

void Viewer::OnMouse(wxMouseEvent& evt){
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

		existsUpdate = true;
		UpdateCamera();

	}else if(evt.MiddleIsDown() || evt.RightIsDown()){

		Vector3d dispV = camRotT*Vector3d(0,(double)dx * CAMERASPEED , -(double)dy * CAMERASPEED);
		worldV = prevWorldV+dispV;
		existsUpdate = false;
		UpdateCamera();
	}

	if(loading){ loading=false; ResetGL(); }
}

void Viewer::render(wxPaintEvent& WXUNUSED(evt)){
    if(!IsShown()) return;
    wxGLCanvas::SetCurrent();
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
	DrawGLScene();
}

void Viewer::InitGL(){
	loading=false;
	existsUpdate=false;
	doCollisions=false;
	Move=false;
	pflag=false;
	threadCounter=0;
	gridActive=true;
	redrawFlag = true;
	gridActive = true;
	mouseLDown = false;
	mouseRDown = false;
	mouseMDown = false;
	mouseCaptured = false;

	xInit=0; yInit=0; x=0; y=0;

	redrawCount=0;

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
	UpdateCamera();
}

void Viewer::ResetGL(){
    GetClientSize(&w, &h);
	SetCurrent();
    glViewport(0, 0, (GLint) w, (GLint) h);

	glFlush();
	glFinish();

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glPolygonMode (GL_FRONT, GL_FILL);
	gluPerspective(45.0f,(GLdouble)w/(GLdouble)h,0.1f,15.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if(world != 0 && world->camRadius != 0){
		camRadius = world->camRadius;
		camRotT = world->camRotT;
		camT = camRotT;
		camT.translate(Vector3d(camRadius, 0, 0));
		worldV = world->worldV;

		prevCamT = camT;
		prevWorldV = worldV;

		gridColor = world->gridColor;
		backColor = world->backColor;

		cout << "Viewer: " << worldV[0] << " " << worldV[1] << " " << worldV[2] << endl;
	}else{
		camRotT = AngleAxis<double> (DEG2RAD(-30), Vector3d(0, 1, 0));
		prevCamT = camRotT;
		worldV = Vector3d(0, 0, 0);
		prevWorldV = worldV;
		camRadius = defaultCamRadius;

		backColor = Vector3d(0,0,0);
		gridColor = Vector3d(.5,.5,.0);
	}

	setClearColor();
	UpdateCamera();
}

void Viewer::setClearColor(){
	glClearColor((float)backColor[0],(float)backColor[1],(float)backColor[2],1.0f);
	float FogCol[3]={(float)backColor[0],(float)backColor[1],(float)backColor[2]};
	glFogfv(GL_FOG_COLOR,FogCol);
}

void Viewer::addGrid(){
	glEnable(GL_FOG);
	double sizeX=100.0f;
	double sizeY=100.0f;
	double inc=.5f;
	double d = .005;

	double startX=-sizeX/2.f;
	double endX  =sizeX/2.f;
	double startY=-sizeY/2.f;
	double endY  =sizeY/2.f;

	GLfloat grid2x2[2][2][3] = {
		{{startX, startY, 0.0}, {endX, startY, 0.0}},
		{{startX, endY, 0.0}, {endX, endY, 0.0}}
	};

	GLfloat *grid = &grid2x2[0][0][0];

    glEnable(GL_MAP2_VERTEX_3);
    glMap2f(GL_MAP2_VERTEX_3,
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

