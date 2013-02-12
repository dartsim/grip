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
#include <robotics/World.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

using namespace std;

/**
 * @function drawWorld
 * @brief Draw World and everything inside it: Robots + Objects
 */
void Viewer::drawWorld() { 

  // Draw Objects
  for( unsigned int i = 0; i < mWorld->getNumObjects(); i++ ) {
    for( int j = 0; j < mWorld->getObject(i)->getNumNodes(); j++ ) {
      //drawNode(mWorld->getObject(i)->getNode(j), mWorld->getObject(i)->getNode(j)->getColliding());
      // Get worldTransform
      Eigen::Matrix4d poseMatrix = mWorld->getObject(i)->getNode(j)->getWorldTransform();    
      // Get visTransform
      Eigen::Matrix4d visTransform = mWorld->getObject(i)->getNode(j)->getShape()->getVisTransform();
      Eigen::Matrix4d poseFinal = poseMatrix*visTransform;
      
      Transform<double,3,Affine> pose;
      pose.setIdentity();
      pose.matrix() = poseFinal;  
      drawModel( mWorld->getObject(i)->getNode(j)->getShape()->getVizMesh(), &pose, mWorld->getObject(i)->getNode(j)->getColliding() );
    }
  }

  // Draw Robot
  for( unsigned int i = 0; i < mWorld->getNumRobots(); i++ ) {
    for( unsigned int j = 0; j < mWorld->getRobot(i)->getNumNodes(); j++ ) {
      //drawNode(mWorld->getRobot(i)->getNode(j), mWorld->getRobot(i)->getNode(j)->getColliding());
      // Get worldTransform
      Eigen::Matrix4d poseMatrix = mWorld->getRobot(i)->getNode(j)->getWorldTransform();
      Eigen::Matrix4d visTransform = mWorld->getRobot(i)->getNode(j)->getShape()->getVisTransform();
      Eigen::Matrix4d poseFinal = poseMatrix*visTransform;
      
      Transform<double,3,Affine> pose;
      pose.matrix() = poseFinal;  
      drawModel( mWorld->getRobot(i)->getNode(j)->getShape()->getVizMesh(), &pose, mWorld->getRobot(i)->getNode(j)->getColliding() );
    }
  }  
  
}

/**
 * @function drawModel
 */
void Viewer::drawModel( const aiScene* _model, Transform<double, 3, Affine> *_pose, bool collisionFlag )
{
   if( _model == NULL ) return;

   if(check_for_collisions && collisionFlag){
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(1.0f, .1f, .1f);
	}
   
   glPushMatrix();
   glMultMatrixd( _pose->data() );

   if( _model != NULL ) {
       renderer.drawMesh(Vector3d::Ones(), _model);
   }

   glColor3f(1.0f,1.0f,1.0f);
   glEnable( GL_TEXTURE_2D );
   glDisable(GL_COLOR_MATERIAL);
   glPopMatrix(); 
  
}

void Viewer::drawNode(kinematics::BodyNode* _node, bool _collision)
{
	if(_node == NULL) return;

	Transform<double,3,Affine> pose;
	pose.setIdentity();
	pose.matrix() = _node->getWorldTransform();

	kinematics::Shape *shape = _node->getShape();
	const aiScene* model = shape->getVizMesh();
	if(shape->listIndex == 0) {
		shape->listIndex = renderer.compileDisplayList(Vector3d::Ones(), model);
	}

	if(check_for_collisions && _collision) {
		glDisable(GL_TEXTURE_2D);
		glEnable(GL_COLOR_MATERIAL);
		glColor3f(1.0f, .1f, .1f);
	}

	glPushMatrix();
	glMultMatrixd(pose.data());

	if(shape->getVizMesh() != NULL) {
		if(shape->listIndex) {
			renderer.drawList(shape->listIndex);
		} else {
			renderer.drawMesh(Vector3d::Ones(), model);
		}
	}

	glColor3f(1.0f,1.0f,1.0f);
	glEnable( GL_TEXTURE_2D );
	glDisable(GL_COLOR_MATERIAL);
	glPopMatrix();

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
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glHint(GL_FOG_HINT,GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

	gluLookAt(camT(0,3), camT(1,3), camT(2,3),   // Camera position
              0, 0, 0,                           // Target Vector in scene
              camT(0,2), camT(1,2), camT(2,2));  // Up Vector from Camera pose

	GLfloat position[]= {static_cast<GLfloat>(camT(0,3)),
			static_cast<GLfloat>(camT(1,3)),
			static_cast<GLfloat>(camT(2,3)), 1.0}; // Camera position
	glEnable(GL_TEXTURE_2D);
	glColor4f(1.0f, 1.0f, 1.0f, 0.0f);

	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT1);
	glEnable(GL_LIGHT2);

	GLfloat no_mat[] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat diffuse[] = {.7f, .7f, .7f, .7f};
	GLfloat specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
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

