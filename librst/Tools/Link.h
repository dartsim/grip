#ifndef LINK_H
#define LINK_H

#include <Eigen/Geometry>
#include <Tools/Object.h>
#include <Tools/Constants.h>
#include <vector>
class Robot;

class Link: public Object {
public:

	Eigen::Transform<double, 3, Eigen::Affine> pose;
	Eigen::Transform<double, 3, Eigen::Affine> jTrans;
	Eigen::Vector3d jAxis;

	Robot *robot;
	Link *parent;
	vector<Link*> children;

	enum JointType {
		REVOL, PRISM, FIXED, FREE
	};

	int index;
	JointType jType;
	double jVal;
	double jMin, jMax;

	void updateRelPose();
	void updateAbsPose();
	void updateRecursive(bool fromJoints = false, bool collisions = false);

	void updateParentPose();
	void updateParentPoseRecursive(bool fromJoints = false, bool collisions = false);

	void recursiveSetAncestry(Robot *, Link *);
	void updateRecursiveCOM();

	Link();
	Link(Link &);
	~Link();
};

#endif
