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

#include <kinematics/Joint.h>
#include <kinematics/Dof.h>
#include <kinematics/Transformation.h>
#include <robotics/Robot.h>
#include <robotics/Object.h>

#include "TreeView.h"
#include "GRIPFrame.h"
#include "../Tabs/InspectorTab.h"
#include "GUI.h"

//TOOLBAR PICTURES
#include "icons/robot.xpm"
#include "icons/object.xpm"
#include "icons/prism.xpm"
#include "icons/revol.xpm"
#include "icons/free.xpm"
#include "icons/fixed.xpm"
#include <string>

IMPLEMENT_DYNAMIC_CLASS(TreeView, wxTreeCtrl)

BEGIN_EVENT_TABLE(TreeView, wxTreeCtrl)
	EVT_TREE_SEL_CHANGED(TreeViewHandle,TreeView::OnSelChanged)
END_EVENT_TABLE()

/**
 * @function TreeView
 * @brief Constructor
 * @date 2011-10-13
 */
TreeView::TreeView(wxWindow *parent, const wxWindowID id,
		   const wxPoint& pos, const wxSize& size,
		   long style)
: wxTreeCtrl(parent, id, pos, size, style | wxTR_FULL_ROW_HIGHLIGHT | wxTR_NO_LINES)
{
  imageList = new wxImageList(16,16,true,0);
  
  const int numIcons=6;
  wxImage icons[numIcons];
  icons[0] = wxImage(robot_xpm);
  icons[1] = wxImage(object_xpm);
  icons[2] = wxImage(prism_xpm);
  icons[3] = wxImage(revol_xpm);
  icons[4] = wxImage(free_xpm);
  icons[5] = wxImage(fixed_xpm);
  
  char r,g,b;
  for(int n=0; n<numIcons; n++){
    icons[n].InitAlpha();
    for(int i=0; i<16; i++)
      for(int j=0; j<16; j++){
	r=icons[n].GetRed(i,j);g=icons[n].GetBlue(i,j);b=icons[n].GetGreen(i,j);
	if(r == g && g == b) icons[n].SetAlpha(i,j,255-r);
      }
  }
  
  for(int i=0; i<6; i++){
    imageList->Add(icons[i]);
  }
  
  AssignImageList(imageList);
}


/**
 * @function CreateFromWorld
 * @brief Create a Tree View from the planning::World object (DART)
 * @date 2011-10-13
 */
void TreeView::CreateFromWorld()
{
  
  if( mWorld == NULL ) return;
  
  DeleteAllItems();
  
  rootId = AddRoot( wxT("Root"), -1, -1 );
  TreeViewReturn* ret;
  
  wxTreeItemId hPrev = rootId;
  
  ///-- Add objects to the tree
  for ( unsigned int i = 0; i < mWorld->getNumObjects(); i++ )
    {
      ret = new TreeViewReturn;
      ret->data = mWorld->getObject(i);
      ret->dType = Return_Type_Object;
      hPrev = AppendItem( rootId,
			  wxString( ( mWorld->getObject(i)->getName() ).c_str(), wxConvUTF8),
			  Tree_Icon_Object,
			  -1,
			  ret );
    }

  ///-- Add robot(s) to the tree
  for ( unsigned int i = 0; i < mWorld->getNumRobots(); i++ )
    {
      ret = new TreeViewReturn;
      ret->data = mWorld->getRobot(i);
      ret->dType = Return_Type_Robot;
      hPrev = AppendItem( rootId,
			  wxString( ( mWorld->getRobot(i)->getName() ).c_str(),wxConvUTF8),
			  Tree_Icon_Robot,
			  -1,
			  ret );

      // Start from root Node
      hPrev = AddNodeTree( (dynamics::BodyNodeDynamics*)mWorld->getRobot(i)->getRoot(), hPrev, hPrev, false );
    }
}

/**
 * @function AddNodeTree
 * @brief Add sub-tree from node of Robot
 * @date 2011-10-13
 */
wxTreeItemId TreeView::AddNodeTree( dynamics::BodyNodeDynamics* _node, wxTreeItemId hPrev, wxTreeItemId hParent, bool inChain )
{
  
  TreeViewReturn* ret;
  int iconIndex = Tree_Icon_Object;
  
  int type = _node->getParentJoint()->getJointType();
  switch (type)
    {
      /** Prismatic Joint: 1 DOF */
    case ( kinematics::Joint::J_TRANS ):
      iconIndex = Tree_Icon_Prismatic;
      break;
      /** Revolute Joint: 1 DOF */
    case (kinematics::Joint::J_HINGE ):
      iconIndex = Tree_Icon_Revolute; 
      break;
      /** Floating Joint: 6 DOF */
    case (kinematics::Joint::J_FREEEULER ): 
      iconIndex = Tree_Icon_Free;
      break;
      /** Fixed Joint: 0 DOF */
    case (kinematics::Joint::J_UNKNOWN ):
      iconIndex = Tree_Icon_Fixed;
      break;
    default:
      iconIndex = Tree_Icon_Object;
      break;
    } 
  
  wxTreeItemId newParent = hParent;
  
  if ( _node->getNumChildJoints() == 1) {
    ret = new TreeViewReturn;
    ret->data = _node;
    ret->dType = Return_Type_Node;
    if ( _node->getChildNode(0)->getNumChildJoints() == 1 && !inChain )
      {    /*_node->idNum = */hPrev = newParent = AppendItem( hParent,
							      wxString( string(_node->getName() ).c_str(),wxConvUTF8),
							      iconIndex,
							      -1,
							      ret );
      }else {
      /*_node->idNum = */hPrev = AppendItem( hParent,
					     wxString( string( _node->getName() ).c_str(),wxConvUTF8),
					     iconIndex, 
					     -1,
					     ret );
    }
    hPrev=AddNodeTree( (dynamics::BodyNodeDynamics*)( _node->getChildNode(0) ), hPrev, newParent, true );
    
  }else {
    ret = new TreeViewReturn;
    ret->data = _node;
    ret->dType = Return_Type_Node;
    hPrev = newParent = AppendItem( hParent, 
				    wxString( string( _node->getName() ).c_str(),wxConvUTF8 ),
				    iconIndex,
				    -1,
				    ret );
    for (int i = 0; i < _node->getNumChildJoints(); i++)
      {    /*_node->idNum = */hPrev=AddNodeTree( (dynamics::BodyNodeDynamics*)( _node->getChildNode(i) ), 
						 hPrev, 
						 newParent, 
						 false );
      }
  }
  return hPrev;
}


/**
 * @function OnSelChanged
 * @brief
 */
void TreeView::OnSelChanged(wxTreeEvent& evt) {
  TreeViewReturn* ret = (TreeViewReturn*)GetItemData(evt.GetItem());
  selectedTreeNode = ret;
  evt.Skip();
}


/**
 * @function ExpandAll
 * @brief
 */
void TreeView::ExpandAll() {

  size_t total = GetCount();
  wxTreeItemId curItem = GetFirstVisibleItem();
  for(size_t i=0; i<total; i++){
    Expand(curItem);
    curItem = GetNextVisible(curItem);
  }
  
}


