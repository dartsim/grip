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
	double stepsize;
private:
	bool copyWorld;
	bool checkPathSegment(int robotId, std::vector<int> linkIds, Eigen::VectorXd config1, Eigen::VectorXd config2);
	void smoothPath(int robotId, std::vector<int> linkIds, std::list<Eigen::VectorXd> &path);
	bool planSingleTreeRrt(std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect = true);
	bool planBidirectionalRrt(std::vector<Link*> links, Eigen::VectorXd start, Eigen::VectorXd goal, std::list<Eigen::VectorXd> &path, bool connect = true);
};