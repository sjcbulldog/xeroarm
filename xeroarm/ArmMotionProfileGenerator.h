#pragma once

#include "ArmDataModel.h"
#include "ArmPath.h"
#include "ArmMotionProfile.h"
#include <QtCore/QVector>

class SplinePair;

class ArmMotionProfileGenerator
{
public:
	ArmMotionProfileGenerator(ArmDataModel &model);

	std::shared_ptr<ArmMotionProfile> generateProfile(std::shared_ptr<ArmPath> path);

private:

	void getSegmentArc(std::shared_ptr<SplinePair> pair, QVector<Pose2d>& results, double t0, double t1, double maxDx, double maxDy, double maxDTheta);

	QVector<std::shared_ptr<SplinePair>>  computeSplinesForPath(std::shared_ptr<ArmPath> path);
	QVector<Pose2d> makeDiscrete(const QVector<std::shared_ptr<SplinePair>>& splines, double maxDx, double maxDy, double maxDTheta);
	QVector<Pose2d> makeEqualDistance(const QVector<Pose2d>& points, double diststep);
	std::shared_ptr<ArmMotionProfile> generateTimedProfile(const QVector<Pose2d>& points);

private:
	ArmDataModel& model_;
};

