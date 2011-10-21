/**
 * @file DrawWorld.cpp
 */
#include <kinematics/Shape.h>
#include <kinematics/BodyNode.h>
#include <kinematics/ShapeMesh.h>
//#include <planning/>
#include "DrawWorld.h"
#include <iostream>

using namespace Eigen;

/**
 * @function DrawWorld
 */
void drawWorld() {


  // Draw Objects     
  for( unsigned int i = 0; i < mWorld->mObjects.size(); i++ ) {

    for( unsigned int j = 0; j < mWorld->mObjects[i]->getNumNodes(); j++ ) {
        
      Eigen::Matrix4d poseMatrix =mWorld->mObjects[i]->getNode(j)->getWorldTransform();         
      std::cout << "Object: "<< mWorld->mObjects[i]->getName() << std::endl;
      std::cout<< poseMatrix << std::endl;
      Transform<double,3,Affine> pose;
      pose.setIdentity();
      pose.matrix() = poseMatrix;  
      drawModel( mWorld->mObjects[i]->mModels[0], pose );
    }
  }

  // Draw Robot     
  for( unsigned int i = 0; i < mWorld->mRobots.size(); i++ ) {

    for( unsigned int j = 0; j < mWorld->mRobots[i]->mModels.size(); j++ ) {
       
      int ind = mWorld->mRobots[i]->mModelIndices[j]; 
      Eigen::Matrix4d poseMatrix =mWorld->mRobots[i]->getNode( ind )->getWorldTransform();   
      std::cout << "Robot Node: "<< mWorld->mRobots[i]->getNode(ind)->getName() << std::endl;
      std::cout<<"Post: \n"<< poseMatrix << std::endl;
      
      Transform<double,3,Affine> pose;
      pose.matrix() = poseMatrix;  
      drawModel( mWorld->mRobots[i]->mModels[j], pose );
    }
  }  

}

/**
 * @function drawModel
 */
void drawModel( Model3DS* _model, Eigen::Transform<double, 3, Eigen::Affine> _pose )
{
   if( _model == NULL ) return;
/*
   if(collisionFlag){
		glDisable(GL_TEXTURE_2D);
		glColor3f(1.0f, .1f, .1f);
	}
*/
   glPushMatrix();
   glMultMatrixd( _pose.data() );
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
	if( _model != NULL ) {
		glCallList(_model->modelDL);
		//glCallList(model->colDL);  // For testing collision model
	}

	glDisable(GL_POLYGON_STIPPLE);
	
	glColor3f(1.0f,1.0f,1.0f);
	glEnable( GL_TEXTURE_2D );
	glPopMatrix(); 
  
}
