#pragma once

#include "Pose2dTrajectory.h"
#include <QtCore/QVector>
#include <memory>

class ArmPath;

class ArmMotionProfile
{
public:
	ArmMotionProfile(std::shared_ptr<ArmPath> path, const QVector<Pose2dTrajectory>& traj) {
		path_ = path;
		traj_ = traj;
	}

	std::shared_ptr<ArmPath> path() const {
		return path_;
	}

	const QVector<Pose2dTrajectory>& trajectory() const {
		return traj_;
	}

	const QStringList &trajectoryNames() {
		if (names_.count() == 0) {
			names_.push_back(EPositionName);
			names_.push_back(EVelocityName);
			names_.push_back(EAccelerationName);

			if (traj_.count() > 0) {
				for (int i = 0; i < traj_.at(0).angles().count(); i++) {
					names_.push_back(QString(APositionName) + "-" + QString::number(i));
					names_.push_back(QString(AVelocityName) + "-" + QString::number(i));
					names_.push_back(QString(AAccelerationName) + "-" + QString::number(i));
				}
			}
		}
		return names_;
	}

	double time() const {
		return traj_.at(traj_.count() - 1).time();
	}

	Pose2dTrajectory getByTime(double t);

public:
	static constexpr const char* EPositionName = "eposition";
	static constexpr const char* EVelocityName = "evelocity";
	static constexpr const char* EAccelerationName = "eacceleration";
	static constexpr const char* APositionName = "aposition";
	static constexpr const char* AVelocityName = "avelocity";
	static constexpr const char* AAccelerationName = "aacceleration";

private:
	std::shared_ptr<ArmPath> path_;
	QVector<Pose2dTrajectory> traj_;
	QStringList names_;
};

