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

/** @file RRT.h
 *  @author Jon Scholz
 */

/*
 * RRT.h
 * Compact RRT library used in stock form for finding a valid path n-dimensions,
 *	e.g. to create a trajectory for a robot arm controller.  Here, you
 *	only need the following:
 *		- start and end points expressible as a vector of doubles ("config")
 *		- lower and upper bounds on the search space, also expressible as configs
 *		- a collision function which returns true for invalid state configurations
 *
 * Using this should be as simple as implementing this collision function, either
 * by changing the actual code or just deriving from the RRT class.
 * 	notes:
 *    1) several functions are made virtual to make the class flexible to more
 * 		 general uses of RRT planning.  E.G. in the SPLinTER project I derived
 * 		 a version that replaced tryStep with a function that iterates over
 *       a set of action primitives rather than subtracting vectors...
 *    2) Init_extras function is there to make it easy to add additional
 *    	 initialization steps in derived classes as necessary
 *
 *
 *  Created on: Aug 27, 2009
 *      Author: jscholz
 */

#ifndef RRT_H
#define RRT_H

#include "ANN/ANN.h"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <Eigen/Core>
#include "World.h"
#include "Link.h"

#define RANDNM(N,M) N + ((M-N) * ((double)rand() / ((double)RAND_MAX + 1))) // random # between N&M

// For representing and manipulating state configurations:
#define config Eigen::VectorXd

class RRT {
public:
	RRT();
	~RRT();

	typedef enum {
		STEP_COLLISION,
		STEP_REACHED,
		STEP_PROGRESS
	} StepResult;

	// Fixed initialization code
	void initialize(World* world, vector<Link*> links, config &ic, double ss = 0.02, int mn = 100000, int ll = 1000, double AE = 0);

	void Init_ANN();

	virtual void cleanup();

	World* world;
	vector<Link*> links;

	int ndim;
	double step_size;
	int max_nodes;
	int linear_limit;
	double ANNeps;

	config initConfig; // Container for starting configuration

	config qtmp; // Container for random configuration

	config minconfig; // Container for minimum configuration values
	config maxconfig; // Container for maximum configuration values

	int activeNode;

	std::vector<int> parentVector;		// vector of indices to relate configs in RRT
	std::vector<config> configVector; 	// vector of all visited configs

	// ANN stuff
	int 				linearNNstart;
	int					nPts;					// actual number of data points
	ANNpointArray		dataPts;				// data points
	ANNpoint			queryPt;				// query point
	ANNidxArray			nnIdx;					// near neighbor indices
	ANNdistArray		dists;					// near neighbor distances
	ANNkd_tree*			kdTree;					// search structure

	bool connect();
	bool connect(config target);
	
	StepResult tryStep();

	StepResult tryStep(config qtry);

	// Tries to extend tree towards provided sample (must be overridden for MBP)
	virtual StepResult tryStep(config qtry, int NNidx);

	// Adds qnew to the tree
	int addNode(config &qnew, int parentID);

	// Sets qsamp a random configuration (may be overridden you want to do something else with sampled states)
	virtual config& getRandomConfig();

	// Returns NN to query point
	int getNearestNeighbor(config &qsamp);

	double getGap(config target);

	// traces the path from some node to the initConfig node
	void tracePath(int node, std::vector<config> &path);

	// Implementation-specific function for checking collisions  (must be overridden for MBP)
	virtual bool checkCollisions(config &c);
};



#endif /* RRT_H */
