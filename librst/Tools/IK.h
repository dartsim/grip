#include <Eigen/Core>
#include <vector>
#include "Link.h"
#include "World.h"


class IK {
public:
	IK(World* world, int robotId, int lastLinkId);
	IK(World* world, int robotId, int lastLinkId, const Eigen::Transform<double, 3, Eigen::Affine> &endEffector);
	bool calculate(Eigen::VectorXd &angles, const Eigen::Transform<double, 3, Eigen::Affine>& goal);
private:
	void init(World* world, int robotId, int lastLinkId, const Eigen::Transform<double, 3, Eigen::Affine> &endEffector);
	Link* links[8];
	Eigen::Transform<double, 3, Eigen::Affine> T0;
	Eigen::Transform<double, 3, Eigen::Affine> A;
	Eigen::Transform<double, 3, Eigen::Affine> B;
	Eigen::Transform<double, 3, Eigen::Affine> Te;

	Eigen::Transform<double, 3, Eigen::Affine> T0Inverse;
	Eigen::Transform<double, 3, Eigen::Affine> TeInverse;
};