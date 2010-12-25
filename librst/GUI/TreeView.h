//---------------------------------------------------------------------
//  Copyright (c) 2008 Saul Reynolds-Haertle & Mike Stilman 
//  All Rights Reserved. 
// 
//  Permission to duplicate or use this software in whole or in part 
//  is only granted by consultation with the author.
//
//    Mike Stilman              mstilman@cc.gatech.edu
//
//	  Robotics and Intelligent Machines
//    Georgia Tech
//--------------------------------------------------------------------

#ifndef TREEVIEW_H
#define TREEVIEW_H

#include "wx/wxprec.h"
#ifndef WX_PRECOMP
	#include "wx/wx.h"
#endif

#include "wx/image.h"
#include "wx/imaglist.h"
#include "wx/treectrl.h"

class Robot;
class Link;

#define TreeViewHandle 1001

enum DataType
{
	Return_Type_Object,
	Return_Type_Robot,
	Return_Type_Link,
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

class TreeView : public wxTreeCtrl
{
public:
	TreeView(){}
	TreeView(wxWindow *parent, const wxWindowID id,
                       const wxPoint& pos, const wxSize& size,
                       long style);
	virtual ~TreeView(){}

	void CreateFromWorld();
	wxTreeItemId AddLinkTree(Link*, wxTreeItemId hPrev, wxTreeItemId hParent, bool inChain);

	void OnSelChanged(wxTreeEvent& event);
	void ExpandAll();

	wxImageList* imageList;
	wxTreeItemId rootId;

	DECLARE_DYNAMIC_CLASS(TreeView)
	DECLARE_EVENT_TABLE()
};



#endif