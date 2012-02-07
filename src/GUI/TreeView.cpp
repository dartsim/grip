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
#include <stdio.h>
#include <stdlib.h>

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


void TreeView::CreateFromDatabase() {
	DeleteAllItems();
	rootId = AddRoot(wxT("Root"), -1, -1);
	wxTreeItemId hPrev = rootId;
	TreeViewReturn* ret;

	multimap<key_, config_t>::iterator it;
	hPrev = AppendItem(rootId, wxString("Configs", wxConvUTF8), Tree_Icon_Robot, -1, NULL);

	for (it = d.config_map.begin(); it != d.config_map.end(); it++) {
		ret = new TreeViewReturn;
		ret->data = const_cast<config_t*> (&((*it).second));
		ret->dType = Return_Type_Config;
		ret->dString = (*it).second.key_s();
		AppendItem(hPrev, wxString((*it).second.key_s().c_str(), wxConvUTF8), Tree_Icon_Object, -1, ret);
	}

	hPrev = AppendItem(rootId, wxString("Layers", wxConvUTF8), Tree_Icon_Robot, -1, NULL);
	for (it = d.layer_map.begin(); it != d.layer_map.end(); it++) {
		ret = new TreeViewReturn;
		ret->data = const_cast<config_t*> (&((*it).second));
		ret->dType = Return_Type_Config;
		ret->dString = (*it).second.key_s();
		AppendItem(hPrev, wxString((*it).second.key_s().c_str(), wxConvUTF8), Tree_Icon_Object, -1, ret);
	}

	char pl_name[25];
	hPrev = AppendItem(rootId, wxString("Packlists", wxConvUTF8), Tree_Icon_Robot, -1, NULL);
	for (uint i = 0; i < o.packlist_vector.size(); i++) {
		sprintf(pl_name, "Packlist %d", i);
		ret = new TreeViewReturn;
		ret->data = const_cast<packlist_*>(&(o.packlist_vector[i]));
		ret->dType = Return_Type_Packlist;
		ret->dString.assign(pl_name);
		AppendItem(hPrev, wxString(pl_name, wxConvUTF8), Tree_Icon_Object, -1, ret);
	}
}

/**
 * @function CreateFromWorld
 * @brief Create a Tree View from the planning::World object (DART)
 * @date 2011-10-13
 */
void TreeView::CreateFromWorld() {

	if (mWorld == NULL)
		return;

	DeleteAllItems();

	rootId = AddRoot(wxT("Root"), -1, -1);
	TreeViewReturn* ret;

	wxTreeItemId hPrev = rootId;

	///-- Add objects to the tree
	for (unsigned int i = 0; i < mWorld->mObjects.size(); i++) {
		ret = new TreeViewReturn;
		ret->data = mWorld->mObjects[i];
		ret->dType = Return_Type_Object;
		mWorld->mObjects[i]->mGripID = hPrev = AppendItem(rootId,
				wxString(mWorld->mObjects[i]->mName.c_str(), wxConvUTF8),
				Tree_Icon_Object, -1, ret);
	}

	///-- Add robot(s) to the tree
	for (unsigned int i = 0; i < mWorld->mRobots.size(); i++) {
		ret = new TreeViewReturn;
		ret->data = mWorld->mRobots[i];
		ret->dType = Return_Type_Robot;
		mWorld->mRobots[i]->mGripID = hPrev = AppendItem(rootId,
				wxString(mWorld->mRobots[i]->mName.c_str(), wxConvUTF8),
				Tree_Icon_Robot, -1, ret);
		///-- Add body nodes ( AKA Links ) as sub-trees
		for (int j = 0; j < mWorld->mRobots[i]->getNumNodes(); j++) {
			// Get the first bodyNode (do not consider the 6 default DOF! )
			if (mWorld->mRobots[i]->getNode(j)->getParentNode() == mWorld->mRobots[i]->getRoot()) {
				hPrev = AddNodeTree(mWorld->mRobots[i]->getNode(j), hPrev, hPrev, false);
			}
		}
	}
}

/**
 * @function AddNodeTree
 * @brief Add sub-tree from node of Robot
 * @date 2011-10-13
 */
wxTreeItemId TreeView::AddNodeTree( kinematics::BodyNode* _node, wxTreeItemId hPrev, wxTreeItemId hParent, bool inChain )
{

	TreeViewReturn* ret;
	int iconIndex = Tree_Icon_Object;

        int type = _node->getParentJoint()->getJointType();
	switch (type)
	    {
	        case ( kinematics::Joint::J_TRANS ):
		    iconIndex = Tree_Icon_Prismatic;
		    break;
	        case (kinematics::Joint::J_HINGE ):
		    iconIndex = Tree_Icon_Revolute;
		    break;
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
	    hPrev=AddNodeTree( _node->getChildNode(0), hPrev, newParent, true );

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
      	    {    /*_node->idNum = */hPrev=AddNodeTree( _node->getChildNode(i),
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


