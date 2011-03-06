/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
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
 *
 */

/** @file RRT.cpp
 *  @author Tobias Kunz
 */

#include "RRT.h"
#include "Robot.h"

using namespace std;
using namespace Eigen;

void RRT::initialize(World* world, int robot, vector<int> links, VectorXd &root, double stepSize)
{
	// clean-up from previous
	cleanup();

	this->world = world;
	this->robot = robot;
	this->links = links;

	ndim = links.size();
	this->stepSize = stepSize;

	srand(time(NULL));

	kdTree = kd_create(ndim);
	addNode(root, -1);
}

RRT::~RRT() {
	cleanup();
}

void RRT::cleanup()
{
	links.clear();
	links.resize(0);

	parentVector.clear();
	configVector.clear();
	parentVector.resize(0);
	configVector.resize(0);
}

bool RRT::connect() {
	VectorXd qtry = getRandomConfig();
	return connect(qtry);
}

bool RRT::connect(const VectorXd &target)
{
	int NNidx = getNearestNeighbor(target);
	StepResult result = STEP_PROGRESS;
	int i = 0;
	while(result == STEP_PROGRESS) {
		result = tryStep(target, NNidx);
		NNidx = configVector.size() - 1;
		i++;
	}
	return (result == STEP_REACHED);
}

RRT::StepResult RRT::tryStep() {
	VectorXd qtry = getRandomConfig();
	return tryStep(qtry);
}

RRT::StepResult RRT::tryStep(const VectorXd &qtry) {
	int NNidx = getNearestNeighbor(qtry);
	return tryStep(qtry, NNidx);
}

RRT::StepResult RRT::tryStep(const VectorXd &qtry, int NNidx)
{
	/*
	 * Calculates a new node to grow towards qtry, checks for collisions, and adds
	 * * also maintains distance to goalConfig
	 */

	VectorXd qnear(ndim);
	VectorXd qnew(ndim);
	qnear = configVector[NNidx];

	// Compute direction and magnitude
	Eigen::VectorXd diff = qtry - qnear;
	double edist = diff.norm();

	if(edist < stepSize) {
		return STEP_REACHED;
	}

	// Scale this vector to step_size and add to end of qnear
	double scale = stepSize / edist;
	for (int i=0; i<ndim; ++i){
		qnew[i] = qnear[i] + diff[i] * scale;
	}
	
	if (!checkCollisions(qnew)) {
		addNode(qnew, NNidx);
		return STEP_PROGRESS;
	}
	else {
		return STEP_COLLISION;
	}
}

int RRT::addNode(const VectorXd &qnew, int parentId)
{
	// Update graph vectors
	configVector.push_back(qnew);
	parentVector.push_back(parentId);
	
	uintptr_t id = configVector.size() - 1;
	kd_insert(kdTree, qnew.data(), (void*)id); //&idVector[id]);

	activeNode = id;
	return id;
}

int RRT::getNearestNeighbor(const VectorXd &qsamp)
{
	struct kdres* result = kd_nearest(kdTree, qsamp.data());
	uintptr_t nearest = (uintptr_t)kd_res_item_data(result);
	
	activeNode = nearest;
	return nearest;
}

VectorXd RRT::getRandomConfig()
{
	/*
	 * Samples a random point for qtmp in the configuration space,
	 * bounded by the provided configuration vectors (and returns ref to it)
	 */
	VectorXd config(ndim);
	for (int i = 0; i < ndim; ++i) {
		config[i] = RANDNM(world->robots[robot]->links[links[i]]->jMin, world->robots[robot]->links[links[i]]->jMax);
	}
	return config;
}

double RRT::getGap(const VectorXd &target) {
	return (target - configVector[activeNode]).norm();
}

void RRT::tracePath(int node, std::list<VectorXd> &path, bool reverse)
{
	int x = node;
	
	while(x != -1) {
		if(!reverse) {
			path.push_front(configVector[x]);
		}
		else {
			path.push_back(configVector[x]);
		}
		x = parentVector[x];
	}
}

bool RRT::checkCollisions(const VectorXd &c)
{
	world->robots[robot]->setConf(links, c);
	return world->checkCollisions();
}

const unsigned int RRT::getSize() {
	return configVector.size();
}