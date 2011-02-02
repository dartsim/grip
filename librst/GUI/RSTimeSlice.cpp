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

#include "../Tools/World.h"
#include "../Tools/Robot.h"
#include "../Tools/Link.h"
#include "../Tools/Object.h"
#include "RSTimeSlice.h"
#include <iostream>

RSTimeSlice::RSTimeSlice(World *w) {
	
	oPose.clear();
	rPose.clear();
	rJoints.clear();
	for(unsigned int i = 0; i < w->objects.size(); i++) {
		oPose.push_back(w->objects[i]->absPose);
	}
	for(unsigned int i = 0; i < w->robots.size(); i++) {
		Robot* r = w->robots[i];
		rPose.push_back(r->baseLink->absPose);
		Eigen::VectorXd RJointVec(r->links.size());
		for(unsigned int j = 0; j < r->links.size(); j++) {
			RJointVec[j] = r->links[j]->jVal;
		}
		rJoints.push_back(RJointVec);
	}
}

RSTimeSlice::~RSTimeSlice() {
}

void RSTimeSlice::SetToWorld(World* w) {
	
	for(unsigned int i = 0; i < w->objects.size(); i++) {
		w->objects[i]->absPose = oPose[i];
	}
	for(unsigned int i = 0; i < w->robots.size(); i++) {
		Robot* r = w->robots[i];
		r->baseLink->absPose = rPose[i];
		for(unsigned int j = 0; j < r->links.size(); j++) {
			r->links[j]->jVal = (rJoints[i])[j];
		}
		r->baseLink->updateRecursive(true); // used to say false before jon removed this arg in favor of global check_for_collisions
	}
	
}