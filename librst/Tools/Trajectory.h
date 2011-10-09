#pragma once

#include <list>
#include <vector>
#include <Eigen/Core>

class Trajectory
{
public:
	Trajectory(const std::list<Eigen::VectorXd> &path, const Eigen::VectorXd &maxVelocity, const Eigen::VectorXd &maxAcceleration);
	Eigen::VectorXd getPosition(double time) const;
	Eigen::VectorXd getVelocity(double time) const;
	double getDuration() const;
private:
	std::vector<Eigen::VectorXd> path;
	std::vector<Eigen::VectorXd> velocities;
	std::vector<Eigen::VectorXd> accelerations;
	std::vector<double> durations;
	std::vector<double> blendDurations;
	double duration;
};
