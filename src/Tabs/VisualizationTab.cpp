/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
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

#include "VisualizationTab.h"

// **********************
// STL
#include <wx/wx.h>
#include <iostream>

using namespace std;

// **********************
// GRIP UI stuff
#include <Tabs/AllTabs.h>
#include <GUI/GUI.h>
#include <GUI/GRIPSlider.h>
#include <GUI/GRIPFrame.h>
#include <GRIPApp.h>

// **********************
// Dynamics Stuff
#include <collision/CollisionDetector.h>
#include <dynamics/SkeletonDynamics.h>
#include <dynamics/ContactDynamics.h>
#include <dynamics/BodyNodeDynamics.h>
#include <kinematics/ShapeBox.h>
#include <kinematics/Dof.h>
#include <kinematics/Joint.h>
#include <renderer/LoadOpengl.h>

// **********************
// Drawing Stuff
#include <wx/glcanvas.h>
#include <GUI/Viewer.h>

/** UI Control IDs */
enum DynamicSimulationTabEvents {
    id_checkbox_showcontacts = wxID_HIGHEST,
    id_checkbox_showcollmesh,
    id_checkbox_CMA,
    id_checkbox_CMP,
    id_checkbox_usecollmesh
};

/** Handlers for events **/
BEGIN_EVENT_TABLE(VisualizationTab, wxPanel)
EVT_COMMAND (wxID_ANY, wxEVT_COMMAND_BUTTON_CLICKED, VisualizationTab::OnButton)
EVT_COMMAND (wxID_ANY, wxEVT_GRIP_SLIDER_CHANGE, VisualizationTab::OnSlider)
EVT_CHECKBOX(id_checkbox_showcontacts, VisualizationTab::OnCheckShowContacts)
EVT_CHECKBOX(id_checkbox_showcollmesh, VisualizationTab::OnCheckShowCollMesh)
EVT_CHECKBOX(id_checkbox_CMP, VisualizationTab::OnCheckShowCMP)
EVT_CHECKBOX(id_checkbox_CMA, VisualizationTab::OnCheckShowCMA)
EVT_CHECKBOX(id_checkbox_usecollmesh, VisualizationTab::OnCheckUseCollMesh)
END_EVENT_TABLE()

// Class constructor for the tab: Each tab will be a subclass of GRIPTab
IMPLEMENT_DYNAMIC_CLASS(VisualizationTab, GRIPTab)

/**
 * @function VisualizationTab
 * @brief Constructor
 */
VisualizationTab::VisualizationTab(wxWindow *parent,
                         const wxWindowID id,
                         const wxPoint& pos,
                         const wxSize& size,
                         long style) :
    GRIPTab(parent, id, pos, size, style),
    selectedNode(NULL)
{
    sizerFull = new wxBoxSizer(wxHORIZONTAL);
    wxStaticBox* ss1Box = new wxStaticBox(this, -1, wxT("Display Options"));
    wxStaticBoxSizer* ss1BoxS = new wxStaticBoxSizer(ss1Box, wxVERTICAL);

    checkShowContacts = new wxCheckBox(this, id_checkbox_showcontacts, wxT("Show Contact Forces"));
    checkShowCollMesh = new wxCheckBox(this, id_checkbox_showcollmesh, wxT("Show Collision Mesh"));
    checkShowCMP = new wxCheckBox(this, id_checkbox_CMP, wxT("Show Projected Center of Mass"));//show projected CM
    checkShowCMA = new wxCheckBox(this, id_checkbox_CMA, wxT("Show Actual Center of Mass"));//show actual CM
    checkUseCollMesh = new wxCheckBox(this, id_checkbox_usecollmesh, wxT("Render using Collision Mesh"));

    ss1BoxS->Add(checkShowContacts, 0, wxALL, 1);
    ss1BoxS->Add(checkShowCollMesh, 0, wxALL, 1);
    ss1BoxS->Add(checkShowCMP, 0, wxALL, 1);
    ss1BoxS->Add(checkShowCMA, 0, wxALL, 1);
    ss1BoxS->Add(checkUseCollMesh, 0, wxALL, 1);
    sizerFull->Add(ss1BoxS, 1, wxEXPAND | wxALL, 1);
    SetSizer(sizerFull);
}


/**
 * @function OnButton
 * @brief Handles button events
 */
void VisualizationTab::OnButton(wxCommandEvent & _evt) {
    int slnum = _evt.GetId();
  
    switch( slnum ) {
    default: {
        /** Default */
        printf("Default button \n");
        break;
    }
    }
}

void VisualizationTab::OnCheckShowContacts(wxCommandEvent &evt) {
}

void VisualizationTab::OnCheckShowCollMesh(wxCommandEvent &evt) {
}

void VisualizationTab::OnCheckShowCMP(wxCommandEvent &evt){
}

void VisualizationTab::OnCheckShowCMA(wxCommandEvent &evt){
}

void VisualizationTab::OnCheckUseCollMesh(wxCommandEvent &evt) {
	viewer->useCollMesh = !viewer->useCollMesh;
}

void VisualizationTab::GRIPEventSceneLoaded() {
}

/**
 * @function GRIPEventSimulationBeforeTimeStep
 * @brief 
 */
void VisualizationTab::GRIPEventSimulationBeforeTimestep() {
}

/**
 * @function GRIPEventSimulationAfterTimeStep
 * @brief
 */
void VisualizationTab::GRIPEventSimulationAfterTimestep() {
}

/**
 * @function GRIPEventSimulationStart
 * @brief
 */
void VisualizationTab::GRIPEventSimulationStart() {
}

/**
 * @function GRIPEventRender
 * @brief
 */
void VisualizationTab::GRIPEventRender() {
    glDisable(GL_FOG);
    glEnable(GL_COLOR_MATERIAL);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);

    glLineWidth(1.5f);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    //draw actual center of mass
    if(mWorld!=NULL&& checkShowCMA->IsChecked()){
        glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
        glEnable ( GL_COLOR_MATERIAL );
        glColor3f(1.0f,0.0f,0.0f);
        for(int x =0 ; x<mWorld->getNumSkeletons();x++){
            glPushMatrix();
            GLUquadricObj * quadric1 = gluNewQuadric();
            Eigen::Vector3d cm1Pos = mWorld->getSkeleton(x)->getWorldCOM();
            glTranslatef(cm1Pos(0),cm1Pos(1),cm1Pos(2));
            gluSphere(quadric1,0.1,5,5);
            gluDeleteQuadric(quadric1);
            glPopMatrix();
        }
    }
    //draw projected center of mass
    if(mWorld!=NULL&& checkShowCMP->IsChecked()){
        glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
        glEnable ( GL_COLOR_MATERIAL );
        glColor3f(1.0f,0.0f,0.0f);
        for(int x =0 ; x<mWorld->getNumSkeletons();x++){
            glPushMatrix();
            GLUquadricObj * quadric2 = gluNewQuadric();
            Eigen::Vector3d cm2Pos = mWorld->getSkeleton(x)->getWorldCOM();
            glTranslatef(cm2Pos(0),cm2Pos(1),0.0);
            gluSphere(quadric2,0.1,5,5);
            gluDeleteQuadric(quadric2);
            glPopMatrix();

        }
    }
    // draw contact points
	if (checkShowContacts->IsChecked() && mWorld && mWorld->getCollisionHandle()) {
        // some preprocessing. calculate vector lengths and find max
        // length, scale down the force measurements, and figure out
        // which contact points involve to the selected body nodes
        int nContacts = mWorld->getCollisionHandle()->getCollisionChecker()->getNumContacts();
        vector<Eigen::Vector3d> vs(nContacts);
        vector<Eigen::Vector3d> fs(nContacts);
        vector<float> lens(nContacts);
        vector<bool> selected(nContacts);
        float maxl = 0;
        for (int k = 0; k < nContacts; k++) {
            collision::Contact contact = mWorld->getCollisionHandle()->getCollisionChecker()->getContact(k);
            vs[k] = contact.point;
            fs[k] = contact.force.normalized() * .1 * log(contact.force.norm());
            lens[k] = (vs[k] - fs[k]).norm();
            if (lens[k] > maxl) maxl = lens[k];
            selected[k] = false;
            if (contact.collisionNode1->getBodyNode() == selectedNode || contact.collisionNode2->getBodyNode() == selectedNode) {
                selected[k] = true;
            }
        }
        Eigen::Vector3d v;
        Eigen::Vector3d f;
        Eigen::Vector3d vf;
        Eigen::Vector3d arrowheadDir;
        Eigen::Vector3d arrowheadBase;
        glBegin(GL_LINES);
        for (int k = 0; k < nContacts; k++) {
            if (selected[k]) {
                glColor3d(0.0, 1.0, 0.0);
            }
            else {
                glColor3d(lens[k] / (2 * maxl) + .5, 0.0, 0.0);
            }
            v = vs[k];
            f = fs[k];
            vf = v + f;
            arrowheadDir = v.cross(f).normalized() * .0075;
            arrowheadBase = vf - f.normalized() * .02;
            glVertex3f(v[0], v[1], v[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(arrowheadBase[0] + arrowheadDir[0], arrowheadBase[1] + arrowheadDir[1], arrowheadBase[2] + arrowheadDir[2]);
            glVertex3f(vf[0], vf[1], vf[2]);
            glVertex3f(arrowheadBase[0] - arrowheadDir[0], arrowheadBase[1] - arrowheadDir[1], arrowheadBase[2] - arrowheadDir[2]);
        }
        glEnd();
    }

    // draw collision meshes
    if (checkShowCollMesh->IsChecked() && mWorld && selectedNode && selectedNode->getCollisionShape()) {
        renderer::RenderInterface* ri = &viewer->renderer;
        kinematics::BodyNode* cnode = selectedNode;
        kinematics::ShapeMesh* shapeMesh = dynamic_cast<kinematics::ShapeMesh *>(selectedNode->getCollisionShape());
        //FIXME: Use OpenGLRenderInterface calls to avoid code duplication.
        if(shapeMesh) {
        	const aiScene* sc = shapeMesh->getMesh();

        	if (sc != NULL) {
        		int verts = 0;
        		const aiNode* nd = sc->mRootNode;

        		// put in the proper transform
        		glPushMatrix();
        		double M[16];
        		Eigen::Matrix4d worldTrans = selectedNode->getWorldTransform();
        		for(int i=0;i<4;i++)
        			for(int j=0;j<4;j++)
        				M[j*4+i] = worldTrans(i, j);
        		glMultMatrixd(M);

        		for (unsigned int n = 0; n < nd->mNumMeshes; ++n) {
        			const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];
        			for (unsigned int t = 0; t < mesh->mNumFaces; ++t) {
        				const struct aiFace* face = &mesh->mFaces[t];
        				glBegin(GL_LINE_STRIP);
        				for(unsigned int i = 0; i < face->mNumIndices; i++) {
        					int index = face->mIndices[i];
        					glColor4d(0.0, 0.0, 1.0, 1.0);
        					if(mesh->mNormals != NULL)
        						glNormal3fv(&mesh->mNormals[index].x);
        					glVertex3fv(&mesh->mVertices[index].x);
        					verts++;
        				}
        				glEnd();
        			}
        		}
        		glPopMatrix();
        	}

        	glPopMatrix();
        	glEnd();
        }
    }
}

/**
 * @function OnSlider
 * @brief Handles slider changes
 */
void VisualizationTab::OnSlider(wxCommandEvent &evt) {
}

// This function is called when an object is selected in the Tree View or other
// global changes to the GRIP world. Use this to capture events from outside the tab.
void VisualizationTab::GRIPStateChange() {
    if(selectedTreeNode==NULL){
        return;
    }

    switch (selectedTreeNode->dType) {
    case Return_Type_Robot:
        selectedNode = ((dynamics::SkeletonDynamics*)selectedTreeNode->data)->mRoot;
        break;
    case Return_Type_Node:
        selectedNode = (dynamics::BodyNodeDynamics*)selectedTreeNode->data;
        break;
    default:
        fprintf(stderr, "someone else's problem.");
        assert(0);
        exit(1);
    }
    int type = 0;
    wxCommandEvent evt(wxEVT_GRIP_UPDATE_AND_RENDER,GetId());
    evt.SetEventObject(this);
    evt.SetClientData((void*)&type);
    GetEventHandler()->AddPendingEvent(evt);
}


