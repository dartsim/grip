#include <Eigen/Core>
#include <vector>
#include <list>
#include "Link.h"
#include "World.h"
#include "RRT.h"
#include <iostream>
#include "Robot.h"

//#ifndef PATH_PLANNER
//#define PATH_PLANNER

template <class R = RRT>
class PathPlanner {
public:
	PathPlanner(); // You need to call one of the other constructors before the object is usable.
	PathPlanner(World& world, bool copyWorld = true, double stepsize = 0.1 );
	~PathPlanner();
	double stepsize;
	World* world;
	bool planPath(int robotId, const std::vector<int> &linkIds, const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd> &path, bool bidirectional = true, bool connect = true, bool smooth = true, unsigned int maxNodes = 0) const;
	bool checkPathSegment(int robotId, const std::vector<int> &linkIds, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2) const;
	void smoothPath(int robotId, std::vector<int> links, std::list<Eigen::VectorXd> &path) const;
        void smoothPath2( int robotId, std::vector<int> linkIds, std::list<Eigen::VectorXd> &path ) const;
private:
	bool copyWorld;
	bool planSingleTreeRrt(int robot, const std::vector<int> &links, const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const;
	bool planBidirectionalRrt(int robot, const std::vector<int> &links, const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const;
	inline void randomInRange(double min, double max);
};


template <class R>
PathPlanner<R>::PathPlanner() : copyWorld(false), world(NULL) {}

template <class R>
PathPlanner<R>::PathPlanner(World& world, bool copyWorld, double stepSize) {
	this->copyWorld = copyWorld;
	if(copyWorld) {
		this->world = new World(world);
	}
	else {
		this->world = &world;
	}
	stepsize = stepSize;
}

template <class R>
PathPlanner<R>::~PathPlanner() {
	if(this->copyWorld) {
		delete this->world;
	}
}

// true iff collision-free
template <class R>
bool PathPlanner<R>::checkPathSegment(int robotId, const vector<int> &linkIds, const Eigen::VectorXd &config1, const Eigen::VectorXd &config2) const {
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

template <class R>
void PathPlanner<R>::smoothPath(int robotId, std::vector<int> linkIds, list<Eigen::VectorXd> &path) const {
	list<VectorXd>::iterator config1, config2;
	list<VectorXd>::iterator temp = path.begin();
	if(temp == path.end()) return;

	while(true) {
		config1 = temp;
		temp++;
		if(temp == path.end()) return;
		config2 = temp;
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

template <class R>
inline void PathPlanner<R>::randomInRange(double min, double max) {
	return min + ((max-min) * ((double)rand() / ((double)RAND_MAX + 1)));
}

template <class R>
void PathPlanner<R>::smoothPath2( int robotId, std::vector<int> linkIds, list<Eigen::VectorXd> &path ) const
{
   srand(time(NULL));

   int node_1; int node_2; int aux_node;

   int num_points = path.size();
   int num_checks = (int) num_points*1;

   // Number of checks
   for( int i = 0; i < num_checks; i++ )
   {
      if( path.size() < 5 ) { return; } //-- No way we can reduce something leaving out the extremes

      int minNode = 0;
      int maxNode = path.size() - 1;

      node_1 = (int) randomInRange( minNode + 1, maxNode - 1 );

      do{ node_2 = (int) randomInRange( minNode + 1, maxNode - 1 ); } while( node_2 == node_1 );

      if( node_2 < node_1 ) 
      {  aux_node = node_1;
         node_1 = node_2;
         node_2 = aux_node; }
      
      //-- Check
      list<Eigen::VectorXd>::iterator n1 = path.begin();
      list<Eigen::VectorXd>::iterator n2 = path.begin();
      advance( n1, node_1 - 1 );
      advance( n2, node_2 - 1 );

      bool result = checkPathSegment( robotId, linkIds, *n1, *n2 );
      if( result == true )
      { int times = node_2 - node_1 - 1;
        for( int j = 0; j < times; j++ )
        { list<Eigen::VectorXd>::iterator temp = path.begin(); 
          advance( temp, node_1 + 1 );
          path.erase( temp );  }
      }
   }   

}
//-----------------------

template <class R>
bool PathPlanner<R>::planPath(int robotId, const std::vector<int> &links, const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd> &path, bool bidirectional, bool connect, bool smooth, unsigned int maxNodes) const {
	
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

template <class R>
bool PathPlanner<R>::planSingleTreeRrt(int robot, const std::vector<int> &links, const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const {

	R rrt(world, robot, links, start, stepsize);
	R::StepResult result = R::STEP_PROGRESS;
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

template <class R>
bool PathPlanner<R>::planBidirectionalRrt(int robot, const std::vector<int> &links, const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const {
	
	R start_rrt(world, robot, links, start, stepsize);
	R goal_rrt(world, robot, links, goal, stepsize);
	R* rrt1 = &start_rrt;
	R* rrt2 = &goal_rrt;
	
	double smallestGap = DBL_MAX;
	bool connected = false;
	while(!connected) {
		R* temp = rrt1;
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



//#endif
