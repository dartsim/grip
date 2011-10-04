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
 *  @author Tobias Kunz
 */

#ifndef RRT_H
#define RRT_H

#include <vector>
#include <list>
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <Eigen/Core>
#include "World.h"
#include "Link.h"
#include "kdtree/kdtree.h"


class RRT {
public:
	RRT(World* world, int robot, const std::vector<int> &links, const Eigen::VectorXd &root, double stepSize = 0.02);
	virtual ~RRT();

	typedef enum {
		STEP_COLLISION, // Collided with obstacle. No node added.
		STEP_REACHED, // The configuration that we grow to is less than stepSize away from the node we grow from. No node added.
		STEP_PROGRESS // One node added.
	} StepResult;

	World* world;
	int robot;
	std::vector<int> links;

	int ndim;
	double stepSize;

	int activeNode;
	std::vector<int> parentVector;		// vector of indices to relate configs in RRT
	std::vector<Eigen::VectorXd> configVector; 	// vector of all visited configs

	struct kdtree *kdTree;

	bool connect();
	bool connect(const Eigen::VectorXd &target);
	
	StepResult tryStep();

	StepResult tryStep(const Eigen::VectorXd &qtry);

	// Tries to extend tree towards provided sample (must be overridden for MBP)
	virtual StepResult tryStepFromNode(const Eigen::VectorXd &qtry, int NNidx);

	// Adds qnew to the tree
	int addNode(const Eigen::VectorXd &qnew, int parentId);

	// returns a random configuration (may be overridden you want to do something else with sampled states)
	virtual Eigen::VectorXd getRandomConfig();

	// Returns NN to query point
	int getNearestNeighbor(const Eigen::VectorXd &qsamp);

	double getGap(const Eigen::VectorXd &target);

	// traces the path from some node to the initConfig node
	void tracePath(int node, std::list<Eigen::VectorXd> &path, bool reverse = false);

	// Implementation-specific function for checking collisions  (must be overridden for MBP)
	virtual bool checkCollisions(const Eigen::VectorXd &c);

	const unsigned int getSize();

protected:
	double RRT::randomInRange(double min, double max);
};

#endif /* RRT_H */