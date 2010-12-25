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

#include "TreeView.h"
#include "RSTFrame.h"
#include "../Tools/World.h"
#include "../Tools/Robot.h"
#include "../Tools/Link.h"
#include "../Tools/Object.h"
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



void TreeView::CreateFromWorld()
{
	if(world == NULL) return;

	DeleteAllItems();
	rootId = AddRoot(wxT("Root"),-1, -1);
	TreeViewReturn* ret;

	wxTreeItemId hPrev = rootId;

	for (unsigned int i = 0; i < world->objects.size(); i++)
	{
		ret = new TreeViewReturn;
		ret->data = world->objects[i];
		ret->dType = Return_Type_Object;
		world->objects[i]->idNum = hPrev = AppendItem(rootId,wxString(world->objects[i]->name.c_str(),wxConvUTF8),Tree_Icon_Object,-1,ret);
	}
	for (unsigned int i = 0; i < world->robots.size(); i++)
	{
		ret = new TreeViewReturn;
		ret->data = world->robots[i];
		ret->dType = Return_Type_Robot;
		world->robots[i]->idNum = hPrev = AppendItem(rootId,wxString(world->robots[i]->name.c_str(),wxConvUTF8),Tree_Icon_Robot,-1,ret);
		for (unsigned int j = 0; j < world->robots[i]->links.size(); j++)
			if (world->robots[i]->links[j]->parent == 0){
				hPrev = AddLinkTree(world->robots[i]->links[j], hPrev, hPrev,false);
			}
	}
}

wxTreeItemId TreeView::AddLinkTree(Link* l, wxTreeItemId hPrev, wxTreeItemId hParent, bool inChain)
{
	TreeViewReturn* ret;
	int iconIndex;
	switch (l->jType)
	{
	case (Link::PRISM):
		iconIndex = Tree_Icon_Prismatic;
		break;
	case (Link::FIXED):
		iconIndex = Tree_Icon_Fixed;
		break;
	case (Link::FREE):
		iconIndex = Tree_Icon_Free;
		break;
	case (Link::REVOL):
		iconIndex = Tree_Icon_Revolute;
		break;
	default:
		iconIndex = Tree_Icon_Object;
		break;
	}

	wxTreeItemId newParent=hParent;
	if (l->children.size() == 1){
		ret = new TreeViewReturn;
		ret->data = l;
		ret->dType = Return_Type_Link;
		if (l->children[0]->children.size() == 1 && !inChain)
			l->idNum = hPrev = newParent = AppendItem(hParent,wxString(l->name.c_str(),wxConvUTF8),iconIndex,-1,ret);
		else
			l->idNum = hPrev = AppendItem(hParent,wxString(l->name.c_str(),wxConvUTF8),iconIndex,-1,ret);
		hPrev=AddLinkTree(l->children[0], hPrev, newParent, true);
	}else{
		ret = new TreeViewReturn;
		ret->data = l;
		ret->dType = Return_Type_Link;
		hPrev = newParent = AppendItem(hParent,wxString(l->name.c_str(),wxConvUTF8),iconIndex,-1,ret);
		for (unsigned int i = 0; i < l->children.size(); i++)
			l->idNum = hPrev=AddLinkTree(l->children[i], hPrev, newParent, false);
	}
	return hPrev;
}

void TreeView::OnSelChanged(wxTreeEvent& evt) {
	TreeViewReturn* ret = (TreeViewReturn*)GetItemData(evt.GetItem());
	selectedTreeNode = ret;
	evt.Skip();
}

void TreeView::ExpandAll() {
	size_t total = GetCount();
	wxTreeItemId curItem = GetFirstVisibleItem();
	for(size_t i=0; i<total; i++){
		Expand(curItem);
		curItem = GetNextVisible(curItem);
	}
}


