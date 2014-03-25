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

#define protected public

#include <iostream>
#include <wx/wx.h>

#include "InspectorTab.h"

#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>

#include <dart/dynamics/Skeleton.h>
#include <dart/dynamics/BodyNode.h>
#include <dart/dynamics/Joint.h>
#include <dart/dynamics/FreeJoint.h>
#include <dart/dynamics/RevoluteJoint.h>
#include <dart/dynamics/PrismaticJoint.h>
#include <dart/dynamics/GenCoord.h>

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

inline double DEG2RAD(double d)  { return (d * 0.01745329251994329577); }
inline double RAD2DEG(double r)	 { return (r * 57.2957795130823208768); }

/**
 * @function OnSlider
 * @brief Handle slider changes
 * @date 2011-10-13
 */
void InspectorTab::OnSlider(wxCommandEvent &evt) {
  
  dart::dynamics::BodyNode* pBodyNode;
  dart::dynamics::Skeleton* pRobot;
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
    pBodyNode = (dart::dynamics::BodyNode*)(selectedTreeNode->data);
    
    switch(slnum) {
      //-- Change joint value
    case J_SLIDER:
      if(dynamic_cast<dart::dynamics::RevoluteJoint*>(pBodyNode->getParentJoint())) {
        std::vector<int> index(1);
        index[0] = pBodyNode->getParentJoint()->getGenCoord(0)->getSkeletonIndex();
        Eigen::VectorXd config(1);
        config[0] = DEG2RAD(pos);
        pBodyNode->getSkeleton()->setConfig(index, config);
      } 
      else if (dynamic_cast<dart::dynamics::PrismaticJoint*>(pBodyNode->getParentJoint())) {
        std::vector<int> index(1);
        index[0] = pBodyNode->getParentJoint()->getGenCoord(0)->getSkeletonIndex();
        Eigen::VectorXd config(1);
        config[0] = pos;
        pBodyNode->getSkeleton()->setConfig(index, config);
      }
      break;
    default:
      printf("None, you are supposed to show this \n");
      return;
      break;
    }

    sprintf(numBuf,"Joint Change: %7.4f", pos);
    
  }
  //-- If selected : ROBOT
  else if(selected == Return_Type_Robot){
    pRobot = (dart::dynamics::Skeleton*)(selectedTreeNode->data);
    
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

  dart::dynamics::BodyNode* pBodyNode;
  dart::dynamics::Skeleton* pRobot;
  
  Eigen::Matrix<double, 6, 1> pose = Eigen::Matrix<double, 6, 1>::Zero();
  
  string statusBuf;
  string buf,buf2;
  int selected = selectedTreeNode->dType;
  
  //-- Return type Node
  if(selected == Return_Type_Node){
    pBodyNode = (dart::dynamics::BodyNode*)(selectedTreeNode->data);
    
    statusBuf = " Selected Node: " + string( pBodyNode->getName() ) + " of Robot: " + string( pBodyNode->getSkeleton()->getName() );
    buf = "Node: " + string( pBodyNode->getName() );
    itemName->SetLabel(wxString(buf.c_str(),wxConvUTF8));
    
    if( pBodyNode->getParentBodyNode() != NULL ) {
      buf2 = "Parent Node: " + string( pBodyNode->getParentBodyNode()->getName() ) + "   Robot: " + string( pBodyNode->getSkeleton()->getName() );
    } else {
      buf2 = "Parent Node: world (NULL)  Robot: " + string( pBodyNode->getSkeleton()->getName() );
    }
      
    /** If joint is hinge */
    if( dynamic_cast<dart::dynamics::RevoluteJoint*>(pBodyNode->getParentJoint()) ) {
      jSlider->setRange( RAD2DEG( pBodyNode->getParentJoint()->getGenCoord(0)->get_qMin() ),
                         RAD2DEG( pBodyNode->getParentJoint()->getGenCoord(0)->get_qMax() ) );
      jSlider->setValue( RAD2DEG( pBodyNode->getParentJoint()->getGenCoord(0)->get_q() ) );
      jSlider->Show();
    }
    /** If joint is translational */
    else if( dynamic_cast<dart::dynamics::PrismaticJoint*>(pBodyNode->getParentJoint()) ) {
      jSlider->setRange( pBodyNode->getParentJoint()->getGenCoord(0)->get_qMin(),
                         pBodyNode->getParentJoint()->getGenCoord(0)->get_qMax() );
      jSlider->setValue( pBodyNode->getParentJoint()->getGenCoord(0)->get_q() );
      jSlider->Show();
    }
    /** If it is Free Euler and root */
    else if( dynamic_cast<dart::dynamics::BodyNode*>(pBodyNode->getParentJoint()) && !pBodyNode->getParentBodyNode() ) {
      // These are 6 values that must be handled with the XYZ RPY sliders
    }
    /** If joint is unknown -- FIXED */
    else {
      jSlider->Hide();
      /** Nothing here, do not show slider for joint */   
    }
    
    //-- Get XYZ and RPY
    pose = getPoseFromTransform(pBodyNode->getWorldTransform());
  }
  
  //-- Return type Robot
  else if(selected == Return_Type_Robot) {
    
    pRobot = (dart::dynamics::Skeleton*)(selectedTreeNode->data);
    
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

/**
 * @function getRootTransform
 * @brief Return <x,y,z, r, p, y> Remember that get_q() gives you the screw so 
 * do NOT use it directly
 */
Eigen::Matrix<double, 6, 1> InspectorTab::getRootTransform(dart::dynamics::Skeleton* robot) {
  dart::dynamics::Joint *joint = robot->getRootBodyNode()->getParentJoint();
  Eigen::Matrix<double, 6, 1> pose;

  if(joint->getJointType() == dart::dynamics::Joint::FREE) {
    Matrix<double, 6, 1> q = joint->get_q();
    Eigen::Isometry3d Tf = dart::math::expMap( joint->get_q() );
    pose.head<3>() = Tf.translation();
    pose.tail<3>() = dart::math::matrixToEulerXYZ( Tf.linear() );
  }
  else {
    pose = getPoseFromTransform(joint->getTransformFromParentBodyNode());
  }

  return pose;
}

/**
 * @function setRootTransform
 * @brief Set q (SCREW) from pose <x,y,z,r,p,y> 
 */
void InspectorTab::setRootTransform( dart::dynamics::Skeleton* robot, 
				     const Eigen::Matrix<double, 6, 1>& pose ) {
  dart::dynamics::Joint* joint = robot->getRootBodyNode()->getParentJoint();

  if(dynamic_cast<dart::dynamics::FreeJoint*>(joint)) {
    Matrix<double, 6, 1> q;
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = pose.head<3>();
    transform.linear() = dart::math::eulerXYZToMatrix(pose.tail<3>());
    q = dart::math::logMap(transform);
    joint->set_q( q );
  
  }
  else {
    Eigen::Isometry3d transform;
    transform.makeAffine();
    transform.linear() = dart::math::eulerXYZToMatrix(pose.tail<3>());
    transform.translation() = pose.head<3>();
    joint->setTransformFromParentBodyNode(transform);
  }
  
  for (int i = 0; i < robot->getNumBodyNodes(); ++i) {
    robot->getBodyNode(i)->updateTransform();
  }
}

/**
 * @function getPoseTransform
 * @brief Get a vector <x,y,z,r,p,y> from Transform. NO SCREW
 */
Eigen::Matrix<double, 6, 1> InspectorTab::getPoseFromTransform(const Eigen::Isometry3d& tf) {
  Eigen::Matrix<double, 6, 1> pose;
  pose.head<3>() = tf.translation();
  pose.tail<3>() = dart::math::matrixToEulerXYZ(tf.linear());
  return pose;
}
