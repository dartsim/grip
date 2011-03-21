#include <Eigen/Core>
#include <vector>
#include <list>
#include "Link.h"
#include "World.h"

#ifndef PATH_PLANNER
#define PATH_PLANNER

class PathPlanner {
public:
	PathPlanner(); // You need to call one of the other constructors before the object is usable.
	PathPlanner(World& world, bool copyWorld = true);
	~PathPlanner();
	double stepsize;
	World* world;
	bool planPath(int robotId, std::vector<int> linkIds, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool bidirectional = true, bool connect = true, bool smooth = true, unsigned int maxNodes = 0) const;
	bool checkPathSegment(int robotId, std::vector<int> linkIds, Eigen::VectorXd config1, Eigen::VectorXd config2) const;
	void smoothPath(int robotId, std::vector<int> links, std::list<Eigen::VectorXd> &path) const;
private:
	bool copyWorld;
	bool planSingleTreeRrt(int robot, std::vector<int> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const;
	bool planBidirectionalRrt(int robot, std::vector<int> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect, unsigned int maxNodes) const;
};

#endif