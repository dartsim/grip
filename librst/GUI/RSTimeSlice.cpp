//---------------------------------------------------------------------
//  Copyright (c) 2008 Mike Stilman
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

#include "../Tools/World.h"
#include "../Tools/Robot.h"
#include "../Tools/Link.h"
#include "../Tools/Object.h"
#include "RSTimeSlice.h"

RSTimeSlice::RSTimeSlice(World *w){
	/*
	oPose.clear();
	rPose.clear();
	rJoints.clear();
	for(unsigned int i=0; i<w->objects.size(); i++){
		oPose.push_back(w->objects[i]->absPose);
	}
	for(unsigned int i=0; i<w->robots.size(); i++){
		Robot* r = w->robots[i];
		rPose.push_back(r->baseLink->absPose);
		vector<double> RJointVec;
		RJointVec.clear();
		for(unsigned int j=0; j<r->links.size(); j++){
			RJointVec.push_back(r->links[j]->jVal);
		}
		rJoints.push_back(RJointVec);
	}
	*/
}

RSTimeSlice::~RSTimeSlice(){
	
}

void RSTimeSlice::SetToWorld(World* w){
	/*
	for(unsigned int i=0; i<w->objects.size(); i++){
		w->objects[i]->absPose = oPose[i];
	}
	for(unsigned int i=0; i<w->robots.size(); i++){
		Robot* r = w->robots[i];
		r->baseLink->absPose = rPose[i];
		for(unsigned int j=0; j<r->links.size(); j++){
			r->links[j]->jVal = (rJoints[i])[j];
		}
		r->baseLink->updateRecursive(true); // used to say false before jon removed this arg in favor of global check_for_collisions
	}
	*/
}

