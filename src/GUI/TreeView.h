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

#ifndef TREEVIEW_H
#define TREEVIEW_H

#include "wx/wxprec.h"
#ifndef WX_PRECOMP
	#include "wx/wx.h"
#endif

#include "wx/image.h"
#include "wx/imaglist.h"
#include "wx/treectrl.h"

////
#include <dart/dynamics/BodyNode.h> 
///


#define TreeViewHandle 1001

enum DataType
{
	Return_Type_Robot,
	Return_Type_Node,
	Return_Type_Tree_Root,
	Return_Type_Other
};

enum TreeIcon
{
	Tree_Icon_Robot = 0,
	Tree_Icon_Object = 1,
	Tree_Icon_Prismatic = 2,
	Tree_Icon_Revolute = 3,
	Tree_Icon_Free = 4,
	Tree_Icon_Fixed = 5 
};

class TreeViewReturn : public wxTreeItemData
{
public:
    TreeViewReturn(){}

	void* data;
	DataType dType;
};

/**
 * @function TreeView
 * @brief The tree view that you see at the right side of GRIP GUI
 * @date 2011-10-13
 */
class TreeView : public wxTreeCtrl
{
public:
	TreeView(){}
	TreeView(wxWindow *parent, const wxWindowID id,
                       const wxPoint& pos, const wxSize& size,
                       long style);
	virtual ~TreeView(){}

	void CreateFromWorld();
	wxTreeItemId AddNodeTree( dart::dynamics::BodyNode* _node, wxTreeItemId hPrev, wxTreeItemId hParent, bool inChain );

	void OnSelChanged(wxTreeEvent& event);
	void ExpandAll();

	wxImageList* imageList;
	wxTreeItemId rootId;

	DECLARE_DYNAMIC_CLASS(TreeView)
	DECLARE_EVENT_TABLE()
};



#endif
