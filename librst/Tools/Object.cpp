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
	this->treeCOM = copyFrom.treeCOM;
	this->treeMass = copyFrom.treeMass;
	this->idNum = copyFrom.idNum;
	this->inertia = copyFrom.inertia;
	this->mass = copyFrom.mass;
	this->model = copyFrom.model;
	this->movable = copyFrom.movable;
	this->name = copyFrom.name;
	this->primitiveType = copyFrom.primitiveType;
	//for(int i = 0; i < 100; i++)
	//	this->name[i] = copyFrom.name[i];
	//this->quadratic = copyFrom.quadratic;
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
//
//	GLdouble m[16] = { absPose.rot.e(0,0), absPose.rot.e(1,0), absPose.rot.e(2,0), 0,
//					  absPose.rot.e(0,1), absPose.rot.e(1,1), absPose.rot.e(2,1), 0,
//					  absPose.rot.e(0,2), absPose.rot.e(1,2), absPose.rot.e(2,2), 0,
//					  absPose.pos.x, absPose.pos.y, absPose.pos.z, 1 };
 /*   GLdouble m[16] = {absPose(0,0), absPose(1,0), absPose(2,0), 0,
					  absPose(0,1), absPose(1,1), absPose(2,1), 0,
					  absPose(0,2), absPose(1,2), absPose(2,2), 0,
					  absPose(0,3), absPose(1,3), absPose(2,3), 1 };*/
    /*GLdouble m[16] = {1,0,0, 0,
					  0,1,0, 0,
					  0,0,1, 0,
					  absPose(0,3), absPose(1,3), absPose(2,3), 1 };*/
	//glMultMatrixd(m);

	glMultMatrixd(absPose.data());
	//glLoadMatrixd(absPose.data());

	if(individualCOM||treeCOMS||robotFloorCOM||showPrimitive)
	{
		if(individualCOM)
		{
			glPushMatrix();
			glColor3f(0.0f, 1.0f, 0.0f);
			glTranslated(COM(0),COM(1),COM(2));
			DrawSphere(0.02f, 10, 10);
			glPopMatrix();
		}

		if(treeCOMS)
		{
			glPushMatrix();
			glColor3f(1.0f,1.0f,0.0f);
			glTranslated(treeCOM(0),treeCOM(1),treeCOM(2));
			DrawSphere(0.02f,10,10);
			glPopMatrix();
		}

		if(showPrimitive)
		{
			this->drawPrimitive();
		}


	//STIPPLE THE OBJECT IF THE FLAG IS SET

		glEnable(GL_POLYGON_STIPPLE);
		glPolygonStipple(halftone);
		glDisable(GL_TEXTURE_2D);
		glColor3d(0.20f,0.20f,0.20f);
	}

	if(model != NULL) {
		glCallList(model->modelDL);
		//glCallList(model->colDL);
	}

	glDisable(GL_POLYGON_STIPPLE);
	glColor3f(1.0f,1.0f,1.0f);
	glEnable(GL_TEXTURE_2D);
	glPopMatrix();
}

void Object::drawPrimitive()
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

Vector3d Object::toTheseBodyCoordinates(Vector3d point)
{
   //Vector3d result = point - this->absPose.linear();
   Vector3d trans;
   //TODO
   //trans = this->absPose.extractRotation() * result;
   return trans;
}

bool Object::ballFaceCollide(Object* face)
{
	if(face->primitiveType!=RECT_FACE)
		return false;
	//Get Face Normal and Normalization
	Vector3d faceNormal = face->normal;
	//cout<<endl<<"Face Normal:"<<faceNormal<<endl;
	double normalization = sqrt(faceNormal(0)*faceNormal(0) + faceNormal(1)*faceNormal(1) + faceNormal(2)*faceNormal(2));

	//Get Sphere Center
//	Vec3 center = face->toTheseBodyCoordinates(this->absPose.pos);
//	cout<<endl<<"Sphere Center:"<<center<<endl;

	//double distanceNumerator = faceNormal[0]*center[0] + faceNormal[1]*center[1] + faceNormal[2]*center[2] + face->d;

	//cout<<endl<<"Face d:"<<face->d<<endl;
	//cout<<endl<<"Distance Numerator:"<<distanceNumerator<<endl;
	//now we get the signed distance
	//+ means same distance on same side as normal
	//- means the distance from the opposite side of the face
	//double totalDistance = distanceNumerator / normalization;

	//cout<<endl<<"Total distance to face:"<<totalDistance<<endl;
//
//	if(fabs(totalDistance) < (this->radius + threshold))
//		return true;

	return false;
}
