#pragma once

#include "Pose2d.h"

class Pose2dTrajectory : public Pose2d
{
public:
	Pose2dTrajectory();
	Pose2dTrajectory(double x, double y);
	Pose2dTrajectory(double x, double y, const Rotation2d& rot);
	Pose2dTrajectory(const Translation2d& pos);
	Pose2dTrajectory(const Rotation2d& pos);
	Pose2dTrajectory(const Translation2d& pos, const Rotation2d& rot);
	Pose2dTrajectory(const Pose2d& other);
	Pose2dTrajectory(double x, double y, const Rotation2d& rot, double time, double pos, double vel, double accel);
	Pose2dTrajectory(const Translation2d& pos, const Rotation2d& rot, double time, double dist, double vel, double accel);
	Pose2dTrajectory(const Pose2d& other, double time, double pos, double vel, double accel);
	virtual ~Pose2dTrajectory();

	double time() const {
		return time_;
	}

	double distance() const {
		return position_;
	}

	double velocity() const {
		return velocity_;
	}

	double acceleration() const {
		return acceleration_;
	}


private:
	double time_;
	double position_;
	double velocity_;
	double acceleration_;
};

