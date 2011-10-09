#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "Link.h"
#include "World.h"


class IK {
public:
	IK() {}; // You need to call one of the other constructors before the object is usable.
	IK(World* world, int robotId, int lastLinkId);
	IK(World* world, int robotId, int lastLinkId, const Eigen::Transform<double, 3, Eigen::Affine> &endEffector);
	bool calculate(const Eigen::Transform<double, 3, Eigen::Affine>& goal, double phi, Eigen::VectorXd &angles) const;
	static void anglesFromRotationMatrix(double &theta1, double &theta2, double &theta3, const Eigen::Vector3d &n1, const Eigen::Vector3d &n2, const Eigen::Vector3d &n3, const Eigen::Matrix3d &A);
private:
	void init(World* world, int robotId, int lastLinkId, const Eigen::Transform<double, 3, Eigen::Affine> &endEffector);
	Eigen::Transform<double, 3, Eigen::Affine> transform(Eigen::Vector3d translation);
	Eigen::Transform<double, 3, Eigen::Affine> transform(Eigen::Matrix3d rotation);

	Link* links[8];
	Eigen::Transform<double, 3, Eigen::Affine> T0;
	Eigen::Transform<double, 3, Eigen::Affine> A;
	Eigen::Transform<double, 3, Eigen::Affine> B;
	Eigen::Transform<double, 3, Eigen::Affine> Te;

	Eigen::Transform<double, 3, Eigen::Affine> T0Inverse;
	Eigen::Transform<double, 3, Eigen::Affine> TeInverse;
};