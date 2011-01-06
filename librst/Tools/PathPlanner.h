#include <Eigen/Core>
#include <vector>
#include "Link.h"
#include "World.h"

class PathPlanner {
public:
	bool planPath(World* world, std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::vector<Eigen::VectorXd> &path);
};