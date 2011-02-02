#include <Eigen/Core>
#include <vector>
#include <list>
#include "Link.h"
#include "World.h"

class PathPlanner {
public:
	World* world;
	PathPlanner(World& world, bool copyWorld = true);
	~PathPlanner();
	bool planPath(int robotId, std::vector<int> linkIds, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool bidirectional = true, bool connect = true, bool smooth = true);

private:
	bool copyWorld;
	bool planSingleTreeRrt(std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect = true);
	bool planBidirectionalRrt(std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect = true);
};