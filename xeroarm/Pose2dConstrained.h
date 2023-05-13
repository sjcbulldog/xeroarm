#pragma once

#include "Pose2dTrajectory.h"

class Pose2dConstrained
{
public:
	Pose2dConstrained(int size) : pose2d_(size) {
		pos_ = 0.0;
	}

	void setPosition(double v) {
		pos_ = v;
	}

	double position() const {
		return pos_;
	}

	void setPose(const Pose2dTrajectory& t) {
		pose2d_ = t;
	}

	const Pose2dTrajectory& pose() const {
		return pose2d_;
	}

	void setVelocity(double v) {
		vel_ = v;
	}

	double velocity() const {
		return vel_;
	}

	void setAccelMax(double v) {
		maxaccel_ = v;
	}

	double accelMax() const {
		return maxaccel_;
	}

	void setAccelMin(double v) {
		minaccel_ = v;
	}

	double accelMin() const {
		return minaccel_;
	}

private:
	Pose2dTrajectory pose2d_;
	double pos_;
	double vel_;
	double maxaccel_;
	double minaccel_;
};