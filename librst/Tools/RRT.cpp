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
 *  @author Jon Scholz
 */

#include "RRT.h"
#include "Robot.h"

using namespace std;

RRT::RRT() {
	dataPts = NULL;
}

void RRT::initialize(World* world, vector<Link*> links, config &ic, double ss, int mn, int ll, double AE)
{
	// clean-up from previous
	cleanup();

	this->world = world;
	this->links = links;

	ndim = ic.size();
	step_size = ss;
	max_nodes = mn;
	linear_limit = ll;
	ANNeps = AE;

	initConfig = config(ic);

	qtmp = config(ic);

	parentVector.clear();
	configVector.clear();

	srand(time(NULL));

	Init_ANN();
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

	if (dataPts != NULL) {
		annDeallocPts(dataPts);
		delete[] nnIdx;
		delete[] dists;
		delete kdTree;
		annClose();
	}
}

void RRT::Init_ANN()
{
	int maxPts = max_nodes; // node_limit+1
	int k = 1;

	linearNNstart=0;

	queryPt = annAllocPt(ndim);					// allocate query point
	for(int i=0; i<ndim; i++)
		queryPt[i] = 0.1*i;

	dataPts = annAllocPts(max_nodes, ndim);		// allocate data points
	nnIdx = new ANNidx[k];						// allocate near neighbor indices
	dists = new ANNdist[k];						// allocate near neighbor dists

	nPts = 0;									// counter for number of data points

	addNode(initConfig, -1); 				// Add initConfig config and "-1" to state vectors and ANN

	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// current number of points
		ndim);						// dimension of space
}

bool RRT::connect() {
	config qtry = getRandomConfig();
	return connect(qtry);
}

bool RRT::connect(config target)
{
	int NNidx = getNearestNeighbor(target);
	StepResult result = STEP_PROGRESS;
	int i = 0;
	while(result == STEP_PROGRESS) {
		result = tryStep(target, NNidx);
		NNidx = configVector.size() - 1;
		i++;
	}
	//cout << i << " " << result << endl;
	return (result == STEP_REACHED);
}

RRT::StepResult RRT::tryStep() {
	config qtry = getRandomConfig();
	return tryStep(qtry);
}

RRT::StepResult RRT::tryStep(const config &qtry) {
	int NNidx = getNearestNeighbor(qtry);
	return tryStep(qtry, NNidx);
}

RRT::StepResult RRT::tryStep(const config &qtry, int NNidx)
{
	/*
	 * Calculates a new node to grow towards qtry, checks for collisions, and adds
	 * * also maintains distance to goalConfig
	 */

	config qnear(ndim);
	config qnew(ndim);
	qnear = configVector[NNidx];

	// Compute direction and magnitude
	Eigen::VectorXd diff = qtry - qnear;
	double edist = diff.norm();

	if(edist < step_size) {
		return STEP_REACHED;
	}

	// Scale this vector to step_size and add to end of qnear
	double scale = step_size / edist;
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


int RRT::addNode(config &qnew, int parentID)
{
	/*
	 * Expands RRT by attaching qnew at parentID (and
	 * balances tree)
	 */

	// Update graph vectors
	configVector.push_back(qnew);
	parentVector.push_back(parentID);

	// add points to ANN data set, weighted appropriately (since i don't know how to change their distance metric)
	for(int i=0; i<ndim; i++)
		dataPts[nPts][i] = qnew[i];
	nPts++;

//cout << "PID = " << parentID << endl;
//cout << "cv = " << configVector.back() << endl;
//cout << "bcidx= " << bestConfIDX << endl;

	// after "linear_limit" steps build new tree
	if(configVector.size() - linearNNstart > linear_limit) {
		delete kdTree;
		kdTree = new ANNkd_tree(dataPts, nPts, ndim);
		linearNNstart = configVector.size();
	}

	activeNode = configVector.size() - 1;
	return configVector.size() - 1;
}

config& RRT::getRandomConfig()
{
	/*
	 * Samples a random point for qtmp in the configuration space,
	 * bounded by the provided configuration vectors (and returns ref to it)
	 */
	for (int i = 0; i < ndim; ++i) {
		qtmp[i] = RANDNM(links[i]->jMin, links[i]->jMax);
	}
	return qtmp;
}

int RRT::getNearestNeighbor(const config &qsamp)
{
	/*
	 * Returns ID of config node nearest to qsamp
	 */

	double min = DBL_MAX;
	double sd = 0.0;
	int nearest=0;

	///////////////////////////////////// DEBUG ///////////////////////////////////////[
	//// Just search the linear list, not the KD-tree:
	//for(int i = 0; i < configVector.size(); ++i){
	//	sd = (qsamp - configVector[i]).squaredNorm();

	//	if(sd < min) {
	//		min = sd;
	//		nearest = i;
	//	}
	//}
	///////////////////////////////////// DEBUG ///////////////////////////////////////}


	//First search the linear vector
	for(int i = linearNNstart; i < configVector.size(); ++i){
		sd = (qsamp - configVector[i]).squaredNorm();

		if(sd < min) {
			min = sd;
			nearest = i;
		}
	}

	//Then search the ANN kd-tree
	if(nPts>linear_limit){
		for(int i = 0; i < ndim; ++i)
			queryPt[i] = qsamp[i];

		kdTree->annkSearch(queryPt, 1, nnIdx, dists, ANNeps);

		// take best result from ANN & list
		if (dists[0] < min)
			nearest = nnIdx[0];
	}
	activeNode = nearest;
	return nearest;
}

double RRT::getGap(config target) {
	return (target - configVector[activeNode]).norm();
}

void RRT::tracePath(int node, std::list<config> &path, bool reverse)
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

bool RRT::checkCollisions(config &c)
{
	for(int i = 0; i < links.size(); i++) {
		links[i]->jVal = c[i];
	}
	links[0]->robot->baseLink->updateRecursive(true, true);
	

	for(unsigned int i=0; i < links[0]->robot->links.size(); i++){
		Link* link = links[0]->robot->links[i];
		if(link->parent == NULL) {
			link->updateRecursive(true, true); // always check collisions?
		}
	}

	world->updateRobot(links[0]->robot);

	return world->checkCollisions();
}

