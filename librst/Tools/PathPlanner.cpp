#include <iostream>
#include "PathPlanner.h"
#include "RRT.h"
#include "Robot.h"

using namespace std;
using namespace Eigen;

PathPlanner::PathPlanner(World& world, bool copyWorld) {
	this->copyWorld = copyWorld;
	if(copyWorld) {
		this->world = new World(world);
	}
	else {
		this->world = &world;
	}
	stepsize = 0.1;
}

PathPlanner::~PathPlanner() {
	if(this->copyWorld) {
		delete this->world;
	}
}

// true iff collision-free
bool PathPlanner::checkPathSegment(int robotId, vector<int> linkIds, VectorXd config1, VectorXd config2) const {
	int n = (int)((config2 - config1).norm() / stepsize);
	for(int i = 0; i < n; i++) {
		VectorXd conf = (double)(n - i)/(double)n * config1 + (double)(i)/(double)n * config2;
		world->robots[robotId]->setConf(linkIds, conf, true);
		if(world->checkCollisions()) {
			return false;
		}
	}
	return true;
}

void PathPlanner::smoothPath(int robotId, std::vector<int> linkIds, list<VectorXd> &path) const {
	list<VectorXd>::iterator temp, config1;
	list<VectorXd>::iterator config2 = path.begin();

	while(true) {
		config1 = config2;
		config2++;
		if(config2 == path.end()) return;
		temp = config2;
		config2++;
		if(config2 == path.end()) return;
		
		while(checkPathSegment(robotId, linkIds, *config1, *config2)) {
			path.erase(temp);
			temp = config2;
			config2++;
			if(config2 == path.end()) return;
		}
	}
}

bool PathPlanner::planPath(int robotId, std::vector<int> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool bidirectional, bool connect, bool smooth, unsigned int maxNodes) const {
	
	world->robots[robotId]->setConf(links, start);
	if(world->checkCollisions())
		return false;
	world->robots[robotId]->setConf(links, goal);
	if(world->checkCollisions())
		return false;
	
	bool result;
	if(bidirectional)
		result = planBidirectionalRrt(robotId, links, start, goal, path, connect, maxNodes);
	else
		result = planSingleTreeRrt(robotId, links, start, goal, path, connect, maxNodes);
	if(result && smooth) {
		smoothPath(robotId, links, path);
	}
	return result;
}

bool PathPlanner::planSingleTreeRrt(int robot, std::vector<int> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const {

	RRT rrt;
	rrt.initialize(world, robot, links, start);
	RRT::StepResult result = RRT::STEP_PROGRESS;
	double smallestGap = DBL_MAX;
	while (result != RRT::STEP_REACHED) {
		if(connect) {
			rrt.connect();
			if(rrt.connect(goal)) {
				result = RRT::STEP_REACHED;
			}
		}
		else {
			rrt.tryStep();
			result = rrt.tryStep(goal);
		}
		if(maxNodes > 0 && rrt.getSize() > maxNodes)
			return false;
		double gap = rrt.getGap(goal);
		if(gap < smallestGap) {
			smallestGap = gap;
			cout << "Gap: " << smallestGap << "    Tree size: " << rrt.configVector.size() << endl;
		}
	}

	rrt.tracePath(rrt.activeNode, path);
	return true;
}

bool PathPlanner::planBidirectionalRrt(int robot, std::vector<int> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const {
	
	RRT start_rrt;
	RRT goal_rrt;
	start_rrt.initialize(world, robot, links, start, stepsize);
	goal_rrt.initialize(world, robot, links, goal, stepsize);
	RRT* rrt1 = &start_rrt;
	RRT* rrt2 = &goal_rrt;
	
	double smallestGap = DBL_MAX;
	bool connected = false;
	while(!connected) {
		RRT* temp = rrt1;
		rrt1 = rrt2;
		rrt2 = temp;

		if(connect) {
			rrt1->connect();
			//rrt1->tryStep();
			connected = rrt2->connect(rrt1->configVector[rrt1->activeNode]);
		}
		else {
			rrt1->tryStep();
			connected = (RRT::STEP_REACHED == rrt2->tryStep(rrt1->configVector[rrt1->activeNode]));
		}

		if(maxNodes > 0 && rrt1->getSize() + rrt2->getSize() > maxNodes)
			return false;

		double gap = rrt2->getGap(rrt1->configVector[rrt1->activeNode]);
		if(gap < smallestGap) {
			smallestGap = gap;
			cout << "Gap: " << smallestGap << "    Tree sizes: " << start_rrt.configVector.size() << "/" << goal_rrt.configVector.size() << endl;
		}
	}
	
	start_rrt.tracePath(start_rrt.activeNode, path);
	goal_rrt.tracePath(goal_rrt.activeNode, path, true);
	
	return true;
}