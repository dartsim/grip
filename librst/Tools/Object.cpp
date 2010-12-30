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

#include <Tools/Object.h>
#include <Tools/GL/glcommon.h>
#include <Tools/Model3DS.h>
#include <Tools/GLTools.h>
#include <GUI/Viewer.h>
#include <string>

const double threshold = 0.02;
using namespace Eigen;

Object::Object()
{
	world=NULL;
	collisionFlag = false;
	comFlag = false;
	model = NULL;
	pathname.erase();
	name.erase();
	movable = false;
	primitiveType = NONE;

	//quadratic=gluNewQuadric();				// Create A Pointer To The Quadric Object ( NEW )
	//gluQuadricNormals(quadratic, GLU_SMOOTH);	// Create Smooth Normals ( NEW )
	//gluQuadricTexture(quadratic, GL_TRUE);
}

Object::Object(Object &copyFrom)
{
	this->absPose = copyFrom.absPose;
	this->COM = copyFrom.COM;
	this->idNum = copyFrom.idNum;
	this->inertia = copyFrom.inertia;
	this->mass = copyFrom.mass;
	this->model = copyFrom.model;
	this->movable = copyFrom.movable;
	this->name = copyFrom.name;
	this->primitiveType = copyFrom.primitiveType;
}

Object::~Object()
{
}

Model3DS* Object::LoadModel(string fullname)
{
	model = new Model3DS();
	model->Load(fullname);
	return model;
}


void Object::Draw()
{
	if(model == NULL) return;

	if(collisionFlag){
		glDisable(GL_TEXTURE_2D);
		glColor3f(1.0f, .1f, .1f);
	}

	glPushMatrix();
	glMultMatrixd(absPose.data());

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

	if(model != NULL) {
		glCallList(model->modelDL);
		//glCallList(model->colDL);  // For testing collision model
	}

	glDisable(GL_POLYGON_STIPPLE);
	
	glColor3f(1.0f,1.0f,1.0f);
	glEnable(GL_TEXTURE_2D);
	glPopMatrix();
}



void Object::DrawPrimitive()
{
	switch(this->primitiveType)
	{
	case NONE:
		break;
	case SPHERE:
		{
			glPushMatrix();
			glColor3f(0.0f, 1.0f, 0.0f);
			DrawSphere(this->radius,10, 10);
			glPopMatrix();
		}
		break;
	case RECT_FACE:
		{
			//Eigen::Vector3d objCenter = this->absPose.linear();
			//printf("Object Center is %lf, %lf, %lf\n", objCenter(0), objCenter(1), objCenter(2));
			glPushMatrix();
			glColor3f(0.0f, 1.0f, 0.0f);
			glBegin(GL_QUADS);
			/*	glVertex3f(objCenter[0] + this->vertices[0][0],objCenter[1] + this->vertices[0][1],objCenter[2] + this->vertices[0][2]);
				glVertex3f(objCenter[0] + this->vertices[1][0],objCenter[1] + this->vertices[1][1],objCenter[2] + this->vertices[1][2]);
				glVertex3f(objCenter[0] + this->vertices[2][0],objCenter[1] + this->vertices[2][1],objCenter[2] + this->vertices[2][2]);
				glVertex3f(objCenter[0] + this->vertices[3][0],objCenter[1] + this->vertices[3][1],objCenter[2] + this->vertices[3][2]);*/
				glVertex3f(this->vertices[0](0), this->vertices[0](1),this->vertices[0](2));
				glVertex3f( this->vertices[1](0),this->vertices[1](1),this->vertices[1](2));
				glVertex3f( this->vertices[2](0),this->vertices[2](1),this->vertices[2](2));
				glVertex3f(  this->vertices[3](0),this->vertices[3](1),this->vertices[3](2));
			glEnd(); //end

			glColor3f(1.0, 0.0, 0.0);
			Eigen::Vector3d vec =  this->vertices[2] -  this->vertices[0];
			vec =  this->vertices[0] + ( vec / 2.0 );
			Eigen::Vector3d normal = this->normal;
			Eigen::Vector3d point2 = vec + normal;

			glBegin(GL_LINE_STRIP);
				glVertex3d( vec[0], vec[1], vec[2] );
				glVertex3d( point2[0], point2[1], point2[2] );
			glEnd();

			glPopMatrix();
		}
	default:
		break;
	}
}

