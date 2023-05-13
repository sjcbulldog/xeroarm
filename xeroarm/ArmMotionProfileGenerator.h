#pragma once

#include "ArmDataModel.h"
#include "ArmPath.h"
#include "ArmMotionProfile.h"
#include "Pose2dConstrained.h"
#include <QtCore/QVector>

class SplinePair;

class ArmMotionProfileGenerator
{
public:
	ArmMotionProfileGenerator(ArmDataModel &model);

	std::shared_ptr<ArmMotionProfile> generateProfile(std::shared_ptr<ArmPath> path);

private:

	void getSegmentArc(std::shared_ptr<SplinePair> pair, QVector<Pose2dTrajectory>& results, double t0, double t1, double maxDx, double maxDy, double maxDTheta);

	QVector<std::shared_ptr<SplinePair>>  computeSplinesForPath(std::shared_ptr<ArmPath> path);
	QVector<Pose2dTrajectory> makeDiscrete(const QVector<std::shared_ptr<SplinePair>>& splines, double maxDx, double maxDy, double maxDTheta);
	QVector<Pose2dTrajectory> makeEqualDistance(const QVector<Pose2dTrajectory>& points, double diststep);
	std::shared_ptr<ArmMotionProfile> generateTimedProfile(std::shared_ptr<ArmPath> path, const QVector<Pose2dTrajectory>& points);

	double jointConstrainedVelocity(const Pose2dConstrained& state, const Pose2dConstrained& pred);
	void computeJointValues(Pose2dTrajectory& traj);

private:
	ArmDataModel& model_;
};

