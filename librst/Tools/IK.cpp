/*
 * Copyright (c) 2009, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "IK.h"
#include "Robot.h"
#include <iostream>
using namespace Eigen;

bool intersect3Lines(const Vector3d &o1, const Vector3d &d1, const Vector3d &o2, const Vector3d &d2, const Vector3d &o3, const Vector3d &d3, Vector3d &intersection) {

	// http://www.gamedev.net/topic/19791-faster-3d-line-line-intersection/

	Vector3d cross = d1.cross(d2);
	double crossSquaredNorm = cross.squaredNorm();
	if(crossSquaredNorm < 0.001) {
		return false;
	}
	Matrix3d m;
	m.row(0) = o2 - o1;
	m.row(1) = d2;
	m.row(2) = cross;
	double s = m.determinant() / crossSquaredNorm;
	m.row(1) = d1;
	double t = m.determinant() / crossSquaredNorm;
	Vector3d p1 = o1 + s * d1;
	Vector3d p2 = o2 + t * d2;

	if((p1 - p2).norm() > 0.001) {
		return false;
	}

	Vector3d p3 = o3 + d3 * d3.dot(p1 - o3);
	if((p3 - p1).norm() > 0.001) {
		return false;
	}

	intersection = p1;
	return true;
}

bool intersect3Joints(Link* link1, Link* link2, Link* link3, Vector3d& intersection, Transform<double, 3, Affine>& trans) {
	Vector3d o1, d1, o2, d2, o3, d3;
	trans = link1->jTrans;
	o1 = trans * Vector3d::Zero();
	d1 = trans.linear() * link1->jAxis;
	trans = trans * link2->jTrans;
	o2 = trans * Vector3d::Zero();
	d2 = trans.linear() * link2->jAxis;
	trans = trans * link3->jTrans;
	o3 = trans * Vector3d::Zero();
	d3 = trans.linear() * link3->jAxis;
	return intersect3Lines(o1, d1, o2, d2, o3, d3, intersection);
}

Transform<double, 3, Affine> transform(Vector3d translation) {
	Transform<double, 3, Affine> T;
	T.setIdentity();
	T.translation() = translation;
	return T;
}

Transform<double, 3, Affine> transform(Matrix3d rotation) {
	Transform<double, 3, Affine> T;
	T.setIdentity();
	T.rotation() = rotation;
	return T;
}

void IK::init(World* world, int robotId, int lastLinkId, const Transform<double, 3, Affine> &endEffector) {
	
	Link* link = world->robots[robotId]->links[lastLinkId];
	int i = 7;
	while(i >= 0) {
		if(link->jType == Link::REVOL || i == 0) {
			links[i] = link;
			i--;
		}
		link = link->parent;
	}

	Vector3d S1;
	Transform<double, 3, Affine> trans13;
	if(!intersect3Joints(links[1], links[2], links[3], S1, trans13)) {
		cout << "Error: No shoulder point" << endl;
	}

	Vector3d S2;
	Transform<double, 3, Affine> trans57;
	if(!intersect3Joints(links[5], links[6], links[7], S2, trans57)) {
		cout << "Error: No wrist point" << endl;
	}

	T0 = links[0]->absPose * transform(S1);
	A = transform((Vector3d)(-1.0 * S1)) * trans13 * links[4]->jTrans;
	B = transform(S2) * transform(trans57.rotation());
	Te = B.inverse() * trans57 * endEffector;

	T0Inverse = T0.inverse();
	TeInverse = Te.inverse();
}

IK::IK(World* world, int robotId, int lastLinkId) {
	Transform<double, 3, Affine> endEffector;
	endEffector.setIdentity();
	init(world, robotId, lastLinkId, endEffector);
}


IK::IK(World* world, int robotId, int lastLinkId, const Transform<double, 3, Affine> &endEffector) {
	init(world, robotId, lastLinkId, endEffector);
}


void anglesFromRotationMatrix(double &theta1, double &theta2, double &theta3, const Vector3d &n1, const Vector3d &n2, const Vector3d &n3, const Matrix3d &A) {
	double lambda = std::atan2(n3.cross(n2).dot(n1), n3.dot(n1));
	theta2 = -lambda - acos(n1.transpose() * A * n3);
	theta3 = -std::atan2(n1.transpose() * A * n2, -n1.transpose() * A * n3.cross(n2));
	theta1 = -std::atan2(n2.transpose() * A * n3, -n2.cross(n1).transpose() * A * n3);
}


bool IK::calculate(const Transform<double, 3, Eigen::Affine> &goal, double phi, VectorXd &angles) const {
    
	const Transform<double, 3, Affine> relGoal = T0Inverse * goal * TeInverse; // target relative to shoulder

	const double L1 = A.translation().norm();
	const double L2 = B.translation().norm();
	const double L3 = relGoal.translation().norm();

	if(L3 > L1 + L2) {
		//cout << "Error: Goal too far away" << endl;
		return false;
	}

	const double theta4 = acos((L3*L3 - L2*L2 - L1*L1)/(2*L1*L2));
	const Vector3d n = relGoal.translation().normalized();
	
	const double cosAlpha = (L3*L3 + L1*L1 - L2*L2) / (2*L3*L1);
	const Vector3d c = cosAlpha * L1 * n;
	const double R = sqrt(1 - cosAlpha*cosAlpha) * L1;

	const Vector3d a = T0.inverse() * Vector3d(0.0, 0.0, -1.0);
	const Vector3d u = (a - a.dot(n) * n).normalized();
	const Vector3d v = n.cross(u);

    const Vector3d elbow = c + R*cos(phi)*u + R*sin(phi)*v;

	Transform<double, 3, Affine> Ty;
	Ty = AngleAxis<double>(theta4, links[4]->jAxis);
	Vector3d w = (A * Ty * B).translation();
	
	Vector3d x = A.translation().normalized();
	Vector3d y = (w - w.dot(x) * x).normalized();
	Vector3d z = x.cross(y);

	Vector3d wg = relGoal.translation();
	Vector3d xg = elbow.normalized();
	Vector3d yg = (wg - wg.dot(xg) * xg).normalized();
	Vector3d zg = xg.cross(yg);

	Transform<double, 3, Affine> temp = A * Ty * B;

	Matrix3d R1r;
	R1r.col(0) = x;
	R1r.col(1) = y;
	R1r.col(2) = z;

	Matrix3d R1g;
	R1g.col(0) = xg;
	R1g.col(1) = yg;
	R1g.col(2) = zg;

	Transform<double, 3, Affine> T1 = Transform<double, 3, Affine>(R1g * R1r.transpose());
	
	Transform<double, 3, Affine> T2 = (T1 * A * Ty * B).inverse() * relGoal;

	double theta1, theta2, theta3;
	Vector3d n1 = links[1]->jAxis;
	Vector3d n2 = links[2]->jTrans.linear() * links[2]->jAxis;
	Vector3d n3 = links[2]->jTrans.linear() * links[3]->jTrans.linear() * links[3]->jAxis;
	anglesFromRotationMatrix(theta1, theta2, theta3, n1, n2, n3, T1.rotation());
	
	double theta5, theta6, theta7;
	Vector3d n5 = links[7]->jTrans.inverse().linear() * links[6]->jTrans.inverse().linear() * links[5]->jAxis;
	Vector3d n6 = links[7]->jTrans.inverse().linear() * links[6]->jAxis;
	Vector3d n7 = links[7]->jAxis;
	anglesFromRotationMatrix(theta5, theta6, theta7, n5, n6, n7, T2.rotation());

	angles.resize(7);
	angles[0] = theta1;
	angles[1] = theta2;
	angles[2] = theta3;
	angles[3] = theta4;
	angles[4] = theta5;
	angles[5] = theta6;
	angles[6] = theta7;

	return true;
}
