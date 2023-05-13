#pragma once

#include "Pose2d.h"
#include <QtCore/QVector>
#include <cassert>

class Pose2dTrajectory : public Pose2d
{
public:
	Pose2dTrajectory(int size);
	Pose2dTrajectory(int size, double x, double y);
	Pose2dTrajectory(int size, double x, double y, const Rotation2d& rot);
	Pose2dTrajectory(int size, const Translation2d& pos);
	Pose2dTrajectory(int size, const Rotation2d& pos);
	Pose2dTrajectory(int size, const Translation2d& pos, const Rotation2d& rot);
	Pose2dTrajectory(int size, const Pose2d& other);
	Pose2dTrajectory(const Pose2dTrajectory& other, double time, double pos, double vel, double accel);
	virtual ~Pose2dTrajectory();

	Pose2dTrajectory interpolate(const Pose2dTrajectory& other, double pcnt) const;

	double time() const {
		return time_;
	}

	void setAngles(const QVector<double>& ang) {
		angles_ = ang;
	}

	const QVector<double>& angles() const {
		return angles_;
	}

	void setVelocities(const QVector<double>& vels) {
		assert(vels.count() == angles_.count());
		avelocity_ = vels;
	}

	const QVector<double>& velocities() const {
		return avelocity_;
	}

	void aAccel(int which, double v) {
		aaccel_[which] = v;
	}

	const QVector<double> &aAccel() const {
		return aaccel_;
	}

	void setAaccel(const QVector<double>& v) {
		assert(v.count() == angles_.count());
		aaccel_ = v;
	}

	void setAccel(double v) {
		accel_ = v;
	}

	double accel() const {
		return accel_;
	}

	void setVelocity(double v) {
		vel_ = v;
	}

	double velocity() const {
		return vel_;
	}

	void setPosition(double v) {
		pos_ = v;
	}

	double position() const {
		return pos_;
	}

	void setTime(double v) {
		time_ = v;
	}

private:
	void init(int size);

	QVector<double> interpolate(const QVector<double>& first, const QVector<double>& second, double pcnt) const;

private:
	double time_;
	double pos_;
	double vel_;
	double accel_;
	QVector<double> angles_;
	QVector<double> avelocity_;
	QVector<double> aaccel_;
};

