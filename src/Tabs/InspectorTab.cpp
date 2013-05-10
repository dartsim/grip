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

#include <iostream>
#include <wx/wx.h>

#include "InspectorTab.h"

#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>

#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <kinematics/Transformation.h>

using namespace std;
using namespace Eigen;

//Give each slider a number so we recognize them
enum sliderNames{
	J_SLIDER = 1000,
	X_SLIDER = 1001,
	Y_SLIDER = 1002,
	Z_SLIDER = 1003,
	ROLL_SLIDER = 1004,
	PITCH_SLIDER = 1005,
	YAW_SLIDER = 1006
};

//Add a handler for slider changes
BEGIN_EVENT_TABLE(InspectorTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, InspectorTab::OnSlider)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of wxPanel
IMPLEMENT_DYNAMIC_CLASS(InspectorTab, GRIPTab)

InspectorTab::InspectorTab(wxWindow *parent, const wxWindowID id,
			   const wxPoint& pos, const wxSize& size,
			   long style)
: GRIPTab(parent, id, pos, size, style)
{
  sizerFull = new wxBoxSizer(wxHORIZONTAL);
  
  wxStaticBox* jointBox = new wxStaticBox(this,-1,wxT("Item Information"));
  wxStaticBox* posBox = new wxStaticBox(this,-1,wxT("Position"));
  wxStaticBox* rotBox = new wxStaticBox(this,-1,wxT("Orientation"));
  
  wxStaticBoxSizer* jointBoxS = new wxStaticBoxSizer(jointBox, wxVERTICAL);
  wxStaticBoxSizer* posBoxS = new wxStaticBoxSizer(posBox, wxVERTICAL);
  wxStaticBoxSizer* rotBoxS = new wxStaticBoxSizer(rotBox, wxVERTICAL);
  
  sizerFull->Add(jointBoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(posBoxS, 1, wxEXPAND | wxALL, 6);
  sizerFull->Add(rotBoxS, 1, wxEXPAND | wxALL, 6);
  SetSizer(sizerFull);
  
  // Add the Joint Information and Joint Slider
  itemName = new wxStaticText(this,-1,wxT("Item: (none)"),wxDefaultPosition,wxDefaultSize,wxALIGN_CENTRE);
  parentName = new wxStaticText(this,-1,wxT("Parent: (none)"),wxDefaultPosition,wxDefaultSize,wxALIGN_CENTRE);
  
  jSlider = new GRIPSlider("Joint",-180,180,2000,0,1000,2000,this,J_SLIDER);
  jointBoxS->Add( parentName, 1, wxEXPAND | wxALL, 6 );
  jointBoxS->Add(itemName,1,wxEXPAND | wxALL, 6);
  jointBoxS->Add(jSlider,1,wxEXPAND | wxALL, 6);
  
  xSlider = new GRIPSlider("X",-10,10,500,0,100,500,this,X_SLIDER);
  ySlider = new GRIPSlider("Y",-10,10,500,0,100,500,this,Y_SLIDER);
  zSlider = new GRIPSlider("Z",-10,10,500,0,100,500,this,Z_SLIDER);
  posBoxS->Add( xSlider,1,wxEXPAND | wxALL, 6 );
  posBoxS->Add( ySlider,1,wxEXPAND | wxALL, 6 );
  posBoxS->Add( zSlider,1,wxEXPAND | wxALL, 6 );
  
  
  rollSlider = new GRIPSlider("R",-180,180,500,0,100,500,this,ROLL_SLIDER);
  pitchSlider = new GRIPSlider("P",-180,180,500,0,100,500,this,PITCH_SLIDER);
  yawSlider = new GRIPSlider("Y",-180,180,500,0,100,500,this,YAW_SLIDER);
  rotBoxS->Add(rollSlider,1,wxEXPAND | wxALL, 6);
  rotBoxS->Add(pitchSlider,1,wxEXPAND | wxALL, 6);
  rotBoxS->Add(yawSlider,1,wxEXPAND | wxALL, 6);
  
 }

/**
 * @function OnSlider
 * @brief Handle slider changes
 * @date 2011-10-13
 */
void InspectorTab::OnSlider(wxCommandEvent &evt) {
  
  dynamics::BodyNodeDynamics* pBodyNode;
  dynamics::SkeletonDynamics* pRobot;
  Eigen::Matrix<double, 6, 1> pose;
  pose << 0, 0, 0, 0, 0, 0;
  
  int slnum = evt.GetId();
  double pos = *(double*)evt.GetClientData();
  char numBuf[1000];
  numBuf[0] = '\0';
  //sprintf(numBuf,"");
  if(selectedTreeNode == NULL){ return; }
  
  int selected = selectedTreeNode->dType;
  
  //-- If selected : NODE
  if( selected == Return_Type_Node ){
    pBodyNode = (dynamics::BodyNodeDynamics*)(selectedTreeNode->data);
    
    switch(slnum) {
      //-- Change joint value
    case J_SLIDER:
      if( pBodyNode->getParentJoint()->getJointType() == kinematics::Joint::J_HINGE ) {
	pBodyNode->getParentJoint()->getDof(0)->setValue( DEG2RAD(pos) ); 
      } 
      else if ( pBodyNode->getParentJoint()->getJointType() == kinematics::Joint::J_TRANS ) {
	pBodyNode->getParentJoint()->getDof(0)->setValue( pos ); 
      }
      break;
    default:
      printf("None, you are supposed to show this \n");
      return;
      break;
    }
    /// Update the robot or object (both Skeletons)
    update((dynamics::SkeletonDynamics*)pBodyNode->getSkel());
    sprintf(numBuf,"Joint Change: %7.4f", pos);
    
  }
  //-- If selected : ROBOT
  else if(selected == Return_Type_Robot){
    pRobot = (dynamics::SkeletonDynamics*)(selectedTreeNode->data);
    
    switch(slnum) {
    case X_SLIDER:
    case Y_SLIDER:
    case Z_SLIDER:
    case ROLL_SLIDER:
    case PITCH_SLIDER:
    case YAW_SLIDER: {
      pose(0) = xSlider->pos;
      pose(1) = ySlider->pos;
      pose(2) = zSlider->pos;
      pose(3) = DEG2RAD(rollSlider->pos);
      pose(4) = DEG2RAD(pitchSlider->pos);
      pose(5) = DEG2RAD(yawSlider->pos);
      setRootTransform(pRobot, pose);
    }
      break;
    default:
      return;
      break;
    } // end switch 

  } // end pRobot
  
    if(frame!=NULL) frame->SetStatusText(wxString(numBuf,wxConvUTF8));

    mWorld->checkCollision(true);
    viewer->DrawGLScene();
}

/**
 * @function GRIPStateChange
 * @brief  This function is called when an object is selected in the Tree View
 * the sliders are shown / hidden and set according to the type of object
 * that is selected and its properties
 * @date 2011-10-13
 */
void InspectorTab::GRIPStateChange() {

  if( selectedTreeNode == NULL ){
    itemName->SetLabel(wxString("Item: None",wxConvUTF8));
    parentName->Hide();
    jSlider->Hide();
    return;
  }

  dynamics::BodyNodeDynamics* pBodyNode;
  dynamics::SkeletonDynamics* pRobot;
  
  Eigen::Matrix<double, 6, 1> pose;
  pose << 0, 0, 0, 0, 0, 0;
  
  string statusBuf;
  string buf,buf2;
  int selected = selectedTreeNode->dType;
  
  //-- Return type Node
  if(selected == Return_Type_Node){
    pBodyNode = (dynamics::BodyNodeDynamics*)(selectedTreeNode->data);
    
    statusBuf = " Selected Node: " + string( pBodyNode->getName() ) + " of Robot: " + string( pBodyNode->getSkel()->getName() );
    buf = "Node: " + string( pBodyNode->getName() );
    itemName->SetLabel(wxString(buf.c_str(),wxConvUTF8));
    
    if( pBodyNode->getParentNode() != NULL ) {
      buf2 = "Parent Node: " + string( pBodyNode->getParentNode()->getName() ) + "   Robot: " + string( pBodyNode->getSkel()->getName() );
    } else {
      buf2 = "Parent Node: world (NULL)  Robot: " + string( pBodyNode->getSkel()->getName() );
    }
      
      /** If joint is hinge */
      if( pBodyNode->getParentJoint()->getJointType() == kinematics::Joint::J_HINGE ) {
	
	jSlider->setRange( RAD2DEG( pBodyNode->getParentJoint()->getDof(0)->getMin() ),
			   RAD2DEG( pBodyNode->getParentJoint()->getDof(0)->getMax() ) );
	jSlider->setValue( RAD2DEG( pBodyNode->getParentJoint()->getDof(0)->getValue() ) );
	
	jSlider->Show();
	
	/** If joint is translational */
      } else if( pBodyNode->getParentJoint()->getJointType() == kinematics::Joint::J_TRANS ) {
	
	jSlider->setRange( pBodyNode->getParentJoint()->getDof(0)->getMin(),
			   pBodyNode->getParentJoint()->getDof(0)->getMax() );
	jSlider->setValue( pBodyNode->getParentJoint()->getDof(0)->getValue() );
	
	jSlider->Show();
	/** If it is Free Euler and root */
      } else if( pBodyNode->getParentJoint()->getJointType() == kinematics::Joint::J_FREEEULER  && pBodyNode->getParentNode() == NULL ) {
	// These are 6 values that must be handled with the XYZ RPY sliders
	/** If joint is unknown -- FIXED */
      } else {
	jSlider->Hide();
	/** Nothing here, do not show slider for joint */   
      }
      

    
    //-- Get XYZ and RPY
    Eigen::Matrix4d tf = pBodyNode->getWorldTransform();
    pose(0) = tf(0,3); 
    pose(1) = tf(1,3); 
    pose(2) = tf(2,3);
    pose(3) = atan2( tf(2,1), tf(2,2) );
    pose(4) = -asin( tf(2,0) );
    pose(5) = atan2( tf(1,0), tf(0,0) );  
    
  }
  
  //-- Return type Robot
  else if(selected == Return_Type_Robot) {
    
    pRobot = (dynamics::SkeletonDynamics*)(selectedTreeNode->data);
    
    statusBuf = " Selected Robot: " + pRobot->getName();
    buf = "Robot: " + pRobot->getName();
    itemName->SetLabel(wxString(buf.c_str(),wxConvUTF8));
    parentName->Show();
    parentName->SetLabel(wxString("",wxConvUTF8));
    jSlider->Hide();
    
    //-- Get XYZ and RPY (pose)
    pose = getRootTransform(pRobot);
  }
  
  frame->SetStatusText(wxString(statusBuf.c_str(),wxConvUTF8));
  sizerFull->Layout(); 
  
  xSlider->setValue( pose(0), false);
  ySlider->setValue( pose(1), false);
  zSlider->setValue( pose(2), false);
  
  rollSlider->setValue( RAD2DEG( pose(3) ), false );
  pitchSlider->setValue( RAD2DEG( pose(4) ), false );
  yawSlider->setValue( RAD2DEG( pose(5) ), false );
  
}

Eigen::Matrix<double, 6, 1> InspectorTab::getRootTransform(dynamics::SkeletonDynamics* robot) {
  kinematics::Joint *joint = robot->getRoot()->getParentJoint();

  assert(joint->getNumTransforms() >= 4);
  assert(joint->getTransform(0)->getType() == kinematics::Transformation::T_TRANSLATE);
  assert(joint->getTransform(1)->getType() == kinematics::Transformation::T_ROTATEZ);
  assert(joint->getTransform(2)->getType() == kinematics::Transformation::T_ROTATEY);
  assert(joint->getTransform(3)->getType() == kinematics::Transformation::T_ROTATEX);

  Eigen::Matrix<double, 6, 1> pose;
  pose(0) = joint->getTransform(0)->getDof(0)->getValue();
  pose(1) = joint->getTransform(0)->getDof(1)->getValue();
  pose(2) = joint->getTransform(0)->getDof(2)->getValue();
  pose(3) = joint->getTransform(3)->getDof(0)->getValue();
  pose(4) = joint->getTransform(2)->getDof(0)->getValue();
  pose(5) = joint->getTransform(1)->getDof(0)->getValue();
  return pose;
}

void InspectorTab::setRootTransform(dynamics::SkeletonDynamics* robot, const Eigen::Matrix<double, 6, 1>& pose) {
  kinematics::Joint* joint = robot->getRoot()->getParentJoint();

  assert(joint->getNumTransforms() >= 4);
  assert(joint->getTransform(0)->getType() == kinematics::Transformation::T_TRANSLATE);
  assert(joint->getTransform(1)->getType() == kinematics::Transformation::T_ROTATEZ);
  assert(joint->getTransform(2)->getType() == kinematics::Transformation::T_ROTATEY);
  assert(joint->getTransform(3)->getType() == kinematics::Transformation::T_ROTATEX);

  joint->getTransform(0)->getDof(0)->setValue(pose(0));
  joint->getTransform(0)->getDof(1)->setValue(pose(1));
  joint->getTransform(0)->getDof(2)->setValue(pose(2));
  joint->getTransform(1)->getDof(0)->setValue(pose(5));
  joint->getTransform(2)->getDof(0)->setValue(pose(4));
  joint->getTransform(3)->getDof(0)->setValue(pose(3));
  joint->updateStaticTransform();
  update(robot);
}

void InspectorTab::update(dynamics::SkeletonDynamics* robot) {
  for(int i = 0; i < robot->getNumNodes(); i++) {
    robot->getNode(i)->updateTransform();
  }
  
  for(int i=0; i < robot->getNumNodes(); i++) {
    robot->getNode(i)->updateFirstDerivatives();
  }
}
