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

#include "Camera.h"
#include "wx/sizer.h"
#include "GUI.h"
#include <wx/glcanvas.h>

#include <Tools/GL/glcommon.h>
#include <Tools/Constants.h>
#include <iostream>
#include <robotics/World.h>
#include <robotics/Object.h>
#include <robotics/Robot.h>

#include <GL/glut.h>
using namespace std;

/// Draw World and everything inside it: Robots + Objects
void Camera::drawWorld () { 

  // Draw Objects	
  for(int i = 0; i < mWorld->getNumObjects(); i++ ) {
		for( int j = 0; j < mWorld->getObject(i)->getNumNodes(); j++ ) {
			
			Eigen::Matrix4d poseMatrix =mWorld->getObject(i)->getNode(j)->getWorldTransform();		 
			Transform<double,3,Affine> pose;
			pose.setIdentity();
			pose.matrix() = poseMatrix;  
			drawModel( mWorld->getObject(i)->getNode(j)->getShape()->getVizMesh(), &pose, mWorld->getObject(i)->getNode(j)->getColliding() );
		}
  }

  // Draw Robot	 
  for(int i = 0; i < mWorld->getNumRobots(); i++ ) {
		for(int j = 0; j < mWorld->getRobot(i)->getNumNodes(); j++ ) {
			Eigen::Matrix4d poseMatrix =mWorld->getRobot(i)->getNode(j)->getWorldTransform();   
			Transform<double,3,Affine> pose;
			pose.matrix() = poseMatrix;  
			drawModel( mWorld->getRobot(i)->getNode(j)->getShape()->getVizMesh(), &pose, mWorld->getRobot(i)->getNode(j)->getColliding() );
		}
  }  

}
/**
 * @function drawModel
 */
void Camera::drawModel( const aiScene* _model, Transform<double, 3, Affine> *_pose, bool collisionFlag )
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


/**
 * @function shown
 */
void Camera::shown(wxShowEvent& WXUNUSED(evt)){
	GetClientSize(&width, &height);
	#ifndef __WXMOTIF__
	if (GetContext())
	#endif
	{
		SetCurrent();
		glViewport(0, 0, (GLint) width, (GLint) height);
	}
	DrawGLScene();
}

/**
 * @function DrawGLScene
 */
int Camera::DrawGLScene()
{
	// Handle the run conditions
	if(!IsShown() || !GetParent()->IsShown()) return false;
	else if(mWorld == NULL) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		SwapBuffers();
		printf("%c[%d;%dmCamera: Cannot draw because a world is not loaded.%c[%dm\n",27,1,33,27,0);
		return false;
	}
	else if(cameraNode == NULL) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		SwapBuffers();
		printf("%c[%d;%dmCamera: Cannot draw because the world does not contain a camera.%c[%dm\n",27,1,33,27,0);
		return false;
	}

	// Setup the view options
	glPolygonMode (GL_FRONT, GL_FILL);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
//	glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);

	// Get the camera orientation and the location
	Matrix4d camTf = cameraNode->getWorldTransform();

	Vector3d loc (camTf(0,3), camTf(1,3), camTf(2,3));
	Vector3d up (camTf(0,1), camTf(1,1), camTf(2,1));
	Vector3d target = Vector3d(camTf(0,0), camTf(1,0), camTf(2,0)) + loc;

	// Set the camera orientation and the location in OpenGL
	gluLookAt(loc(0),loc(1),loc(2), 				// Camera position
			  target(0), target(1), target(2), 	// Target Vector in scene
			  up(0), up(1), up(2));  						// Up Vector from Camera pose

	// Setup lighting 
	glEnable(GL_TEXTURE_2D);
	glColor4f(1.0f,1.0f,1.0f,0.0f);
	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT1);
	glEnable(GL_LIGHT2);

	float no_mat[] = {0.0f, 0.0f, 0.0f, 1.0f};
	float diffuse[] = {.7f, .7f, .7f, .7f};
	float specular[] = {0.5f, 0.5f, 0.5f, 1.0f};
	glLightfv(GL_LIGHT1, GL_AMBIENT, no_mat);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, specular);

	GLfloat HeadlightAmb[4] = {0.0f, 0.0f, 0.0f, 1.0f};
	GLfloat HeadlightDif[4] = {1.0f, 1.0f, 1.0f, 0.0f};
	GLfloat HeadlightSpc[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	glLightfv(GL_LIGHT2, GL_AMBIENT, HeadlightAmb);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, HeadlightDif);
	glLightfv(GL_LIGHT2, GL_SPECULAR, HeadlightSpc);

	float ambRefl[] = {0.2f, 0.2f, 0.2f, 1.0f}; // default
	glMaterialfv(GL_FRONT, GL_AMBIENT, ambRefl);
	float diffRefl[] = {0.8f, 0.8f, 0.8f, 1.0f}; // default
	glMaterialfv(GL_FRONT, GL_DIFFUSE, diffRefl);
	float specRefl[] = {1.0f, 1.0f, 1.0f, 1.0f}; // default
	glMaterialfv(GL_FRONT, GL_SPECULAR, specRefl);

	// Draw the world if available
	if( mWorld != NULL ) { 
		drawWorld(); 
  }

	// Swap the front and back buffers and return
	glFlush();
	SwapBuffers();

	return TRUE;
}

/// Handles the resize of the canvas
void Camera::resized(wxSizeEvent& evt){

	// To stop the unused parameter warning for evt.
	(void) evt;	

	// Do not bother with redrawing if not shown
	if(!IsShown() || !GetParent()->IsShown()) return;

	// Set the drawing options	
	GetClientSize(&width, &height);
	#ifndef __WXMOTIF__
	if (GetContext())
	#endif
	{
		SetCurrent();
		glViewport(0, 0, (GLint) width, (GLint) height);
	}
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f,(GLdouble)width/(GLdouble)height,0.1f,15.0f);//100.0f);


	// Redraw the scene
	DrawGLScene();
	//sceneChanged = true;
}

/// Renders the scene
void Camera::render(wxPaintEvent& WXUNUSED(evt)){
	if(!IsShown()) return;
	wxGLCanvas::SetCurrent();
	wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event
//	printf("Render\n");
//	sceneChanged = true;
	DrawGLScene();

}

/// Initializes the openGL options and the class options
void Camera::InitGL(){

	// Set the width and the height of the image
	GetClientSize(&width, &height);
	#ifndef __WXMOTIF__
	if (GetContext())
	#endif
	{
		Show();
		SetCurrent();
		glViewport(0, 0, (GLint) width, (GLint) height);
	}

	// Set the shading model, background color, max depth
	glShadeModel(GL_SMOOTH);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(100.0f);

	// Enable various useful options
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glDepthFunc(GL_LEQUAL);
//	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	// Set the perspective projection
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glPolygonMode (GL_FRONT, GL_FILL);
	gluPerspective(45.0f,(GLdouble)width/(GLdouble)height,0.1f,15.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//printf("Init GL\n");
	//sceneChanged = true;
}

/// The event table for rendering and resizing the camera image
BEGIN_EVENT_TABLE(Camera, wxGLCanvas)
	EVT_SHOW(Camera::shown)
  EVT_PAINT(Camera::render)
  EVT_SIZE(Camera::resized)
	EVT_IDLE(Camera::OnIdle)
END_EVENT_TABLE()
