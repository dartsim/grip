//---------------------------------------------------------------------
//  Copyright (c) 2009 Mike Stilman
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
#include <wx/wx.h>
#include <GUI/Viewer.h>
#include <GUI/GUI.h>
#include <GUI/RSTSlider.h>
#include <GUI/RSTFrame.h>
#include <iostream>
using namespace std;
#include <Tools/World.h>
#include <Tools/Robot.h>
#include <Tools/Link.h>
#include <Tools/Object.h>
#include <Tools/Constants.h>


#include "SimulationTab.h"
#include <Tabs/AllTabs.h>
#include <RSTApp.h>
//#include "../Tools/Boxer.h"
double vel_t_1=0.0;
//vector<string> boxerJointList;	//used for list of joints selected from the GUI
//string	boxerEndEffector;	//used for selected end effecor from the GUI
/* Quick intro to adding tabs:
 * 1- Copy template cpp and header files and replace with new class name
 * 2- include classname.h in AllTabs.h, and use the ADD_TAB macro to create it
 */

// Control IDs (used for event handling - be sure to start with a non-conflicted id)
enum simulationTabEvents {
	button_ShowPrimitive = 6000,
	button_addJoint,
	button_clearJointList,
	button_addEndEffector
};
//Add a handler for slider changes
BEGIN_EVENT_TABLE(SimulationTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, SimulationTab::OnButton)
END_EVENT_TABLE ()

// Class constructor for the tab: Each tab will be a subclass of RSTTab
IMPLEMENT_DYNAMIC_CLASS(SimulationTab, RSTTab)

SimulationTab::SimulationTab(wxWindow *parent, const wxWindowID id,
		const wxPoint& pos, const wxSize& size, long style) :
	RSTTab(parent, id, pos, size, style) {
	sizerFull = new wxBoxSizer(wxHORIZONTAL);

	// Create Static boxes - these are the outlines you see on the inspector tab - a nice way to organize things
	wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Punch Configuration"));
	wxStaticBox* ss2Box = new wxStaticBox(this, -1, wxT(""));

	// Create sizers for these static boxes
	wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);
	wxStaticBoxSizer* ss2BoxS = new wxStaticBoxSizer(ss2Box, wxHORIZONTAL);
	wxBoxSizer *confSizer = new wxBoxSizer(wxVERTICAL);
	confSizer->Add(new wxButton(this, button_addJoint, wxT("&add joint to list")),
		0,
		wxALL,
		1);
	
		confSizer->Add(new wxButton(this, button_addEndEffector, wxT("&select end effector")),
		0,
		wxALL,
		1);
	confSizer->Add(new wxButton(this, button_clearJointList, wxT("&clear joint list")),
		0,
		wxALL,
		1);
	ss2BoxS->Add(new wxButton(this, button_ShowPrimitive, wxT("&Show Primitive")),
		1,
		wxGROW);

	ss1BoxS->Add(confSizer,1,wxALIGN_NOT,0);
/*
	wxBoxSizer *xSizer = new wxBoxSizer(wxHORIZONTAL); // annoying hack to get checkboxes close together
	
	
	xSizer->Add(new wxButton(this, button_minusX, wxT("- X")),
			0, // make horizontally unstretchable
			wxALL, // make border all around (implicit top alignment)
			1); // set border width to 1, so start buttons are close together
	xSizer->Add(new wxButton(this, button_plusX, wxT("+ X")),
			0, // make horizontally unstretchable
			wxALL, // make border all around (implicit top alignment)
			1); // set border width to 1, so start buttons are close together

	wxBoxSizer *ySizer = new wxBoxSizer(wxHORIZONTAL);
	ySizer->Add(new wxButton(this, button_minusY, wxT("- Y")),
		0,
		wxALL,
		1);
	ySizer->Add(new wxButton(this, button_plusY, wxT("+ Y")),
		0,
		wxALL,
		1);
	
	wxBoxSizer *zSizer = new wxBoxSizer(wxHORIZONTAL);
	zSizer->Add(new wxButton(this, button_minusZ, wxT("- Z")),
		0,
		wxALL,
		1);
	zSizer->Add(new wxButton(this, button_plusZ, wxT("+ Z")),
		0,
		wxALL,
		1);

	ss2BoxS->Add(xSizer,1,wxALIGN_NOT,0);
	ss2BoxS->Add(ySizer,1,wxALIGN_NOT,0);
	ss2BoxS->Add(zSizer,1,wxALIGN_NOT,0);
*/
	// Add the boxes to their respective sizers
	sizerFull->Add(ss2BoxS, 1, wxEXPAND | wxALL, 6);
	sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 6);
	SetSizer(sizerFull);
}

// Handle Button Events
void SimulationTab::OnButton(wxCommandEvent &evt) {
//	/*
//	//Robot *r;  //unused
//	//Object* o; //unused
//	//Link* l;   /unused
//	int button_num = evt.GetId();
//	vector<string> rightArm;
//	switch (button_num) {
//
//	case	button_addJoint:
//		{
//			if(selectedTreeNode==NULL){ cout<<endl<<"Must select a link."<<endl; return; }
//
//	/*		int selected = selectedTreeNode->dType;
//			if(selected == Return_Type_Link)
//			{
//				o = (Object*)(selectedTreeNode->data);
//				l = (Link*)(o);
//			}
//			else{ cout<<endl<<"Please select a link to add to the list."<<endl; return;}
//
//			boxerJointList.push_back(l->name);
//			cout<<endl<<"Added "<<l->name<<" to the joint list."<<endl;
//*/	
//	}
//		break;
//	case button_addEndEffector:
//		{
//			//EACH TIME BUTTON IS HIT WE GO THROUGH 1 TIME STEP ITERATION
//			if(world->objects[1]->name=="board2")
//			{
//				double acc_t=0.0;
//				
//				world->objects[1]->vel_t=0.0;
//				world->objects[2]->vel_t=0.0;
//
//				double pos_t;
//				double force = -9.8;
//				double delta_t = 0.01; //1/4 a second
//				double restitution = 1.0;
//				//char why;
//				
//				//cout<<endl<<"Found the board";
//				for(int j=0;j<300;j++)
//				{
//					bool br=false;
//					for(int u=0;u<12;u++) //faces of object 0
//					{
//						for(int v=0;v<12;v++) //vertices of object 1
//						{
//							Vec3 start1 = world->objects[0]->toWorldCoordinates(world->objects[0]->edges[u].start);
//							Vec3 stop1 = world->objects[0]->toWorldCoordinates(world->objects[0]->edges[u].stop);
//							Vec3 start2 = world->objects[1]->toWorldCoordinates(world->objects[1]->edges[v].start);
//							Vec3 stop2 = world->objects[1]->toWorldCoordinates(world->objects[1]->edges[v].stop);
//							//cout<<endl<<"Edge v:"<<v<<" vs. edge u:"<<u<<endl;
//							if(world->objects[0]->edgeEdgeCollide(start1,stop1,start2,stop2))
//							{	
//								cout<<endl<<"Collision on Objec 0 edge:"<<u<<" with Object 1 edge:"<<v<<endl;
//								world->objects[1]->vel_t *= -1.0*restitution;
//								br=true;
//								break;
//							}
//						}
//
//						if(br==true)
//							break;
//					}
//
//				//	if(world->objects[1]->sphereSphereCollide(world->objects[2]))
//				//	{
//						//world->objects[1]->vel_t *= -1.0*restitution;
//				//		world->objects[2]->vel_t *= -1.0*restitution;
//				//	}
//
//
//				acc_t = force/world->objects[1]->mass;
//				world->objects[1]->vel_t = world->objects[1]->vel_t + delta_t * acc_t;
//				pos_t = world->objects[1]->absPose.pos.z + delta_t * world->objects[1]->vel_t;
//				world->objects[1]->absPose.pos.z = pos_t;
//
//				acc_t = force/world->objects[2]->mass;
//				world->objects[2]->vel_t = world->objects[2]->vel_t + delta_t * acc_t;
//				pos_t = world->objects[2]->absPose.pos.z + delta_t * world->objects[2]->vel_t;
//				world->objects[2]->absPose.pos.z = pos_t;
//
//				//vel_t_1 = vel_t;
//				//cout<<endl<<"Acc:"<<acc_t<<" vel:"<<world->objects[1]->vel_t<<" pos:"<<pos_t;
//				viewer->DrawGLScene();
//
//				}
//			}
//		//	if(selectedTreeNode==NULL){ cout<<endl<<"Must select a link."<<endl; return; }
///*
//			int selected = selectedTreeNode->dType;
//			if(selected == Return_Type_Link)
//			{
//				o = (Object*)(selectedTreeNode->data);
//				l = (Link*)(o);
//			}
//			else{ cout<<endl<<"Please select a link for the end effector."<<endl; return;}
//
//			boxerEndEffector = l->name;
//			cout<<endl<<"End Effector Now Set To: "<<l->name<<endl;
//*/
//			}
//		break;
//
//	case button_clearJointList:
//		{
//	//		boxerJointList.clear();	
//	//		boxerEndEffector.clear();
//	//		cout<<endl<<"Jacobian Joint List and End Effector Link have been cleared."<<endl;
//			if(world->objects[1]->sphereFaceCollide(world->objects[0],0))
//				cout<<endl<<"Collision with sphere and rect Face"<<endl;
//			else
//				cout<<endl<<"No collision"<<endl;
//		}
//		break;
//	
//	case button_ShowPrimitive:
//		{
//		cout << "Show Primitive button" << endl;
//		//world->objects[1]->initializeSphere(0.1 , 1.0); //roughly 2.2 lbs
//
//		//world->objects[2]->initializeSphere(0.05 , 0.5);
//
//		//double w=1.5;
//		//double h=1.5;
//		//double d= 1.5;
//		//world->objects[0]->initializeRectFace(w,h);
//		//world->objects[0]->initializeBox(w,h,d,1.0);
//		
//		
//		if(showPrimitive == false)
//		{
//			showPrimitive = true;
//			for(int i =0; i<world->objects.size();i++)
//			{
//				cout<<endl<<"Object:"<<i<<" is of type:"<<world->objects[i]->primitiveType<<endl;
//			}
//		}
//		else
//		{
//			showPrimitive = false;
//		}
//
//		viewer->DrawGLScene();
//
//	/*	if(boxerJointList.empty()||boxerEndEffector.empty())
//		{
//			cout<<"Ensure that the Joint List has joints and that the End Effector has been selected."<<endl;
//			return;
//		}
//*/
//		}
//		break;
//	}
//*/
}

// This function is called when an object is selected in the Tree View or other
// global changes to the RST world. Use this to capture events from outside the tab.
void SimulationTab::RSTStateChange() {
	if(selectedTreeNode==NULL){
		return;
	}

	Object* o;
	Robot* r;
	Link* l;
	string statusBuf;
	string buf, buf2;
	switch (selectedTreeNode->dType) {
	case Return_Type_Object:
		o = (Object*) (selectedTreeNode->data);
		statusBuf = " Selected Object: " + o->name;
		buf = "You clicked on object: " + o->name;
//		sampleText1->SetLabel(wxString(buf.c_str(), wxConvUTF8));
//		sampleText2->Hide();

		break;
	case Return_Type_Robot:
		r = (Robot*) (selectedTreeNode->data);
		statusBuf = " Selected Robot: " + r->name;
		buf = "You clicked on robot: " + r->name;
//		sampleText2->SetLabel(wxString(buf.c_str(), wxConvUTF8));
//		sampleText1->Hide();

		break;
	case Return_Type_Link:
		l = (Link*) (selectedTreeNode->data);
		statusBuf = " Selected Link: " + l->name + " of Robot: "
				+ l->robot->name;
		buf = " Link: " + l->name + " of Robot: " + l->robot->name;
		// Do something here if you want to.  you get the idea...

		break;
    default:
        fprintf(stderr, "Someone else's problem!\n");
        assert(0);
        exit(1);
	}
	//frame->SetStatusText(wxString(statusBuf.c_str(), wxConvUTF8));
	//sizerFull->Layout();
}

void RSTApp::AddTab() {
	ADD_TAB(SimulationTab,wxT("Simulation"))
}


