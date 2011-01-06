#include "PathPlanner.h"
#include "RRT.h"

bool PathPlanner::planPath(World* world, std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::vector<Eigen::VectorXd> &path) {
	RRT rrt;
	
	Eigen::VectorXd lb(links.size());
	Eigen::VectorXd ub(links.size());
	for(int i = 0; i < links.size(); i++) {
		lb[i] = links[i]->jMin;
		ub[i] = links[i]->jMax;
	}

	rrt.initialize(world, links, start, goal, lb, ub);
	while (rrt.bestSD > 0.005) {
		rrt.stepRandom();
		rrt.stepGreedy(rrt.goalConfig);
	}

	rrt.tracePath(path);
	return true;
}