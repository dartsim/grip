#pragma once

#include <list>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

class PathSegment
{
public:
	virtual double getLength() const = 0;
	virtual Eigen::VectorXd getConfiguration(double s) const = 0;
	virtual Eigen::VectorXd getPathVelocity(double s) const = 0;
	virtual Eigen::VectorXd getPathAcceleration(double s) const = 0;
};

class LinearPathSegment : public PathSegment
{
public:
	LinearPathSegment(const Eigen::VectorXd &start, const Eigen::VectorXd &end) :
		start(start),
		end(end),
		length((end-start).norm())
	{
	}

	double getLength() const {
		return length;
	}

	Eigen::VectorXd getConfiguration(double s) const {
		s /= length;
		s = std::max(0.0, std::min(1.0, s));
		return (1.0 - s) * start + s * end;
	}

	Eigen::VectorXd getPathVelocity(double s) const {
		return (end - start) / length;
	}

	Eigen::VectorXd getPathAcceleration(double s) const {
		return Eigen::VectorXd::Zero(start.size());
	}

private:
	Eigen::VectorXd start;
	Eigen::VectorXd end;
	double length;
};

class CircularPathSegment : public PathSegment
{
public:
	CircularPathSegment(const Eigen::VectorXd &start, const Eigen::VectorXd &intersection, const Eigen::VectorXd &end, double maxDeviation) {
		
		Eigen::VectorXd startDirection = (intersection - start).normalized();
		Eigen::VectorXd endDirection = (end - intersection).normalized();

		double distance = std::min((start - intersection).norm(), (end - intersection).norm());
		double angle = acos(startDirection.dot(endDirection));
		distance = std::min(distance, maxDeviation * sin(0.5 * angle) / (1.0 - cos(0.5 * angle)));  // enforce max deviation

		radius = distance / tan(0.5 * angle);
		length = angle * radius;

		center = intersection + (endDirection - startDirection).normalized() * radius / cos(0.5 * angle);
		x = (intersection - distance * startDirection - center).normalized();
		y = end - center;
		y = (y - x * y.dot(x)).normalized();

		// test code
		//double distance1 = (start - intersection).norm();
		//double distance2 = (end - intersection).norm();
		//Eigen::VectorXd testStart = intersection + distance / distance1 * (start - intersection);
		//double startError = (testStart - getConfiguration(0.0)).norm();
		//Eigen::VectorXd testEnd = intersection + distance / distance2 * (end - intersection);
		//double endError = (testEnd - getConfiguration(length)).norm();
		//double deviation = (center - intersection).norm() - radius;
	}

	double getLength() const {
		return length;
	}

	Eigen::VectorXd getConfiguration(double s) const {
		const double angle = s / radius;
		return center + radius * (x * cos(angle) + y * sin(angle));
	}

	Eigen::VectorXd getPathVelocity(double s) const {
		const double angle = s / radius;
		return - x * sin(angle) + y * cos(angle);
	}

	Eigen::VectorXd getPathAcceleration(double s) const {
		const double angle = s / radius;
		return - 1.0 / radius * (x * cos(angle) + y * sin(angle));
	}

private:
	double radius;
	double length;
	Eigen::VectorXd center;
	Eigen::VectorXd x;
	Eigen::VectorXd y;
};


class Path
{
public:
	Path(const std::list<Eigen::VectorXd> &path, double maxDeviation = 0.0);
	~Path();
	double getLength() const;
	Eigen::VectorXd getConfiguration(double s);
	Eigen::VectorXd getPathVelocity(double s);
	Eigen::VectorXd getPathAcceleration(double s);
private:
	PathSegment* getPathSegment(double &s);

	double length;
public:
	std::list<PathSegment*> pathSegments;

};