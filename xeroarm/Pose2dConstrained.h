#pragma once

#include "Pose2dTrajectory.h"

class Pose2dConstrained
{
public:
	Pose2dConstrained(int size) : pose2d_(size), angpos_(size), angvel_(size) {
		pos_ = 0.0;
		maxaccel_ = 0.0;
		minaccel_ = 0.0;
		vel_ = 0.0;
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

	double angVel(int which) const {
		return angvel_.at(which);
	}

	void setAngVel(int which, double v) {
		angvel_[which] = v;
	}

	double angPos(int which) const {
		return angpos_.at(which);
	}

	void setAngPos(int which, double v) {
		angpos_[which] = v;
	}

	double duration() const {
		return duration_;
	}

	void setDuration(double v) {
		duration_ = v;
	}

	int limitingJoint() const {
		return limiting_joint_;
	}

	void setLimitingJoint(int v) {
		limiting_joint_ = v;
	}

private:
	Pose2dTrajectory pose2d_;
	double pos_;
	double vel_;
	double maxaccel_;
	double minaccel_;
	double duration_;
	int limiting_joint_;
	QVector<double> angpos_;
	QVector<double> angvel_;
};
