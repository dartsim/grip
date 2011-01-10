#include "PathPlanner.h"
#include "RRT.h"

bool PathPlanner::planPath(World* world, std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::vector<Eigen::VectorXd> &path, bool connect) {
	
	RRT rrt;
	rrt.initialize(world, links, start);
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
		double gap = rrt.getGap(goal);
		if(gap < smallestGap) {
			smallestGap = gap;
			cout << "Gap: " << smallestGap << "    Tree size: " << rrt.configVector.size() << endl;
		}
	}

	rrt.tracePath(rrt.activeNode, path);
	return true;
}

bool PathPlanner::planPathBidirectional(World* world, std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::vector<Eigen::VectorXd> &path, bool connect) {
	
	RRT start_rrt;
	RRT goal_rrt;
	start_rrt.initialize(world, links, start);
	goal_rrt.initialize(world, links, goal);
	RRT* rrt1 = &start_rrt;
	RRT* rrt2 = &goal_rrt;
	
	double smallestGap = DBL_MAX;
	bool connected = false;
	while(!connected) {
		RRT* temp = rrt1;
		rrt1 = rrt2;
		rrt2 = temp;

		if(connect)
			rrt1->connect();
		else
			rrt1->tryStep();
		
		connected = rrt2->connect(rrt1->configVector[rrt1->activeNode]);
		
		double gap = rrt2->getGap(rrt1->configVector[rrt1->activeNode]);
		if(gap < smallestGap) {
			smallestGap = gap;
			cout << "Gap: " << smallestGap << "    Tree sizes: " << start_rrt.configVector.size() << "/" << goal_rrt.configVector.size() << endl;
		}
	}
	
	start_rrt.tracePath(start_rrt.activeNode, path);
	vector<Eigen::VectorXd> goalPath;
	goal_rrt.tracePath(goal_rrt.activeNode, goalPath);
	path.insert(path.end(), goalPath.rbegin(), goalPath.rend());
	
	return true;
	return false;
}