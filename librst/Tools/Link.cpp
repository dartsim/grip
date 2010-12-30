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

#include <Tools/Link.h>
#include <Tools/Robot.h>
#include <Tools/World.h>

using namespace Eigen;

Link::Link()
{
}

Link::~Link()
{
}

Link::Link(Link &copyFrom): Object(copyFrom)
{
	this->absPose = copyFrom.absPose;
	this->COM = copyFrom.COM;
	this->idNum = copyFrom.idNum;
	this->inertia = copyFrom.inertia;
	this->jAxis = copyFrom.jAxis;
	this->jMax = copyFrom.jMax;
	this->jMin = copyFrom.jMin;
	this->jTrans = copyFrom.jTrans;
	this->jType = copyFrom.jType;
	this->jVal = copyFrom.jVal;
	this->mass = copyFrom.mass;
	this->pose = copyFrom.pose;
	this->model = copyFrom.model;
	this->movable = copyFrom.movable;
	this->name = copyFrom.name;
	this->index = copyFrom.index;
	this->parent = NULL;
	this->robot = NULL;
	this->children.clear();
}

void Link::recursiveSetAncestry(Robot *rootRobot, Link *parentLink)
{
	this->parent = parentLink;
	this->robot = rootRobot;
	this->robot->links.push_back(this);
	for(unsigned int i = 0; i < this->children.size(); i++)
		this->children[i]->recursiveSetAncestry(rootRobot, this);
}

// Updates relative position based on updated joint value
void Link::updateRelPose(){
	Transform<double, 3, Eigen::Affine> jT;
	jT.setIdentity();
	if(jType == PRISM) jT.translate(jAxis*jVal);
	if(jType == REVOL) jT = AngleAxisd(jVal,jAxis);
	pose = jTrans*jT;
}

// Updates absolute pose from relative pose and parent absolute pose
void Link::updateAbsPose(){
	if(parent != NULL){
		absPose = parent->absPose*pose;
	}
}

// Given a new absolute position, propogates effects down the tree
void Link::updateRecursive(bool fromJoints, bool collisions){
	if(collisions)
		this->robot->world->updateCollision(this);

	for(unsigned int i=0; i<children.size(); i++){
		if(fromJoints)
			children[i]->updateRelPose();
		children[i]->updateAbsPose();
		children[i]->updateRecursive(fromJoints, collisions);
	}
}

// Updates absolute parent position based on child absolute position
void Link::updateParentPose(){
	if(parent != NULL) {
		updateRelPose();
		parent->absPose = absPose*pose.inverse(Eigen::Affine);
	}
}

//Same as above but recurses effects throughout the robot
void Link::updateParentPoseRecursive(bool fromJoints, bool collisions){
	if(parent != NULL) {
		updateRelPose();
		parent->absPose = absPose*pose.inverse(Eigen::Affine);
		parent->updateParentPoseRecursive(fromJoints, collisions);
	}else{
		// When all the way up to the base link, come back down
		updateRecursive(fromJoints, collisions);
	}
}
