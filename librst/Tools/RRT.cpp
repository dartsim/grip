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

ostream& operator<<(ostream& os, const config& c) {
	for (int i=0; i<c.size(); ++i){
		os << c[i];
		if (i<c.size()-1)
			os << " ";
	}
	return os;
}

RRT::RRT() {
	dataPts = NULL;
}

void RRT::initialize(World* world, vector<Link*> links, config &ic, config &gc, config &mins, config &maxs, double ss, int mn, int ll, double AE)
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
	goalConfig = config(gc);

	qtmp = config(ic);

	minconfig = config(mins);
	maxconfig = config(maxs);

	parentVector.clear();
	configVector.clear();

	bestConfIDX = -1;
	bestSD = DBL_MAX;

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

	addNode(initConfig, bestConfIDX); 				// Add initConfig config and "-1" to state vectors and ANN

	kdTree = new ANNkd_tree(		// build search structure
		dataPts,					// the data points
		nPts,						// current number of points
		ndim);						// dimension of space
}

void RRT::stepRandom()
{
	/*
	 * Take a step in a random direction (wraps getRandomConfig,
	 * getNearestNeighbor, and tryStep)
	 */
	getRandomConfig();
	int NNidx = getNearestNeighbor(qtmp);
	tryStep(qtmp, NNidx);
}

void RRT::stepGreedy(config &target)
{
	/*
	 * Implement this to take a step towards a specific configuration (also
	 * wraps getRandomConfig, getNearestNeighbor, and addNode)
	 */
	int NNidx = getNearestNeighbor(target);
	tryStep(target, NNidx);
}

void RRT::tryStep(config qtry, int NNidx)
{
	/*
	 * Calculates a new node to grow towards qtry, checks for collisions, and adds
	 * * also maintains distance to goalConfig
	 */

	config qnear(ndim);
	config qnew(ndim);
	qnear = configVector[NNidx];	// sets qnear to the closest configuration to qsamp

	// Compute direction and magnitude
	qnew = qtry - qnear;
	double edist = qnew.norm();

	// Scale this vector to step_size and add to end of qnear
	double scale = (double)step_size / edist;
	for (int i=0; i<ndim; ++i){
		qnew[i] = qnew[i] * scale + qnear[i];
	}

	if (!checkCollisions(qnew)) {
		addNode(qnew, NNidx);

		double sd = (qnew - goalConfig).squaredNorm();

		if (sd < bestSD) {
			bestConfIDX = configVector.size()-1;	// if last node is closest, mark idx as greatestConf
			bestSD = sd;
			bestConfig = configVector[bestConfIDX];
			//cout << "achieved best SD: " << bestSD << endl;
			cout << "achieved best SD: " << bestSD << " (treesize=" << configVector.size() << ")" << endl;
		}
	}
}


void RRT::addNode(config &qnew, int parentID)
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
}

config& RRT::getRandomConfig()
{
	/*
	 * Samples a random point for qtmp in the configuration space,
	 * bounded by the provided configuration vectors (and returns ref to it)
	 */
	for (int i = 0; i < ndim; ++i) {
		qtmp[i] = RANDNM(minconfig[i], maxconfig[i]);
	}
	return qtmp;
}

int RRT::getNearestNeighbor(config &qsamp)
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
	return nearest;
}

void RRT::tracePath(std::vector<config> &path)
{
	path.clear();

	int x = bestConfIDX;

	while(parentVector[x] != -1){
		path.insert(path.begin(), configVector[x]);
		x = parentVector[x];
	}
}

bool RRT::checkCollisions(config &c)
{
	for(int i = 0; i < links.size(); i++) {
		links[i]->jVal = c[i];
	}
	links[0]->robot->baseLink->updateRecursive(true, true);
	return world->checkCollisions();
}

