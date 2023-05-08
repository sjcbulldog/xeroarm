#include "ArmMotionProfileGenerator.h"
#include "SplinePair.h"

ArmMotionProfileGenerator::ArmMotionProfileGenerator(ArmDataModel &model) : model_(model)
{
}

QVector<std::shared_ptr<SplinePair>> ArmMotionProfileGenerator::computeSplinesForPath(std::shared_ptr<ArmPath> path)
{
	QVector<std::shared_ptr<SplinePair>> splines;

	for (int i = 0; i < path->size() - 1; i++) {
		const Pose2d& p1 = path->at(i);
		const Pose2d& p2 = path->at(i + 1);
		auto pair = std::make_shared<SplinePair>(p1, p2);
		splines.push_back(pair);
	}

	return splines;
}

void ArmMotionProfileGenerator::getSegmentArc(std::shared_ptr<SplinePair> pair, QVector<Pose2d>& results,
	double t0, double t1, double maxDx, double maxDy, double maxDTheta)
{
	const Translation2d& p0 = pair->evalPosition(t0);
	const Translation2d& p1 = pair->evalPosition(t1);
	const Rotation2d& r0 = pair->evalHeading(t0);
	const Rotation2d& r1 = pair->evalHeading(t1);
	Pose2d transformation = Pose2d(Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
	Twist2d twist = Pose2d::logfn(transformation);
	if (twist.getY() > maxDy || twist.getX() > maxDx || twist.getTheta() > maxDTheta) {
		getSegmentArc(pair, results, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
		getSegmentArc(pair, results, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
	}
	else {
		results.push_back(pair->evalPose(t1));
	}
}

QVector<Pose2d> ArmMotionProfileGenerator::makeDiscrete(const QVector<std::shared_ptr<SplinePair>>& splines, double maxDx, double maxDy, double maxDTheta)
{
	QVector<Pose2d> results;

	results.push_back(splines[0]->getStartPose());
	for (int i = 0; i < splines.size(); i++)
		getSegmentArc(splines[i], results, 0.0, 1.0, maxDx, maxDy, maxDTheta);

	return results;
}

QVector<Pose2d> ArmMotionProfileGenerator::makeEqualDistance(const QVector<Pose2d>& points, double step)
{
	QVector<double> distances;
	QVector<Pose2d> result;

	static const double kEpsilon = 1e-6;
	double d;

	distances.push_back(0.0);
	for (int i = 1; i < points.size(); i++)
		distances.push_back(points[i].distance(points[i - 1]) + distances[i - 1]);

	int index = 0;
	for (d = 0.0; d <= distances.back(); d += step)
	{
		while (d > distances[index + 1])
			index++;

		double percent = (d - distances[index]) / (distances[index + 1] - distances[index]);
		Pose2d newpt = points[index].interpolate(points[index + 1], percent);
		result.push_back(newpt);
	}

	distances.clear();
	distances.push_back(0.0);
	for (int i = 1; i < result.size(); i++)
		distances.push_back(result[i].distance(result[i - 1]) + distances[i - 1]);

	return result;
}

std::shared_ptr<ArmMotionProfile> ArmMotionProfileGenerator::generateTimedProfile(const QVector<Pose2d>& points)
{
	return nullptr;
}

std::shared_ptr<ArmMotionProfile> ArmMotionProfileGenerator::generateProfile(std::shared_ptr<ArmPath> path)
{
	//
	// Step 1: Generate a set of splines for this path
	//
	auto splines = computeSplinesForPath(path);

	//
	// Step 2: Generate a discrete form of the path where the curvature, x, and y do not deviate
	//         to an amount large enough to misrepresent the path for our purposes
	//
	double maxDx = 1.0;
	double maxDy = 1.0;
	double maxDTheta = 1.0;
	QVector<Pose2d> discrete = makeDiscrete(splines, maxDx, maxDy, maxDTheta);

	//
	// Step 3: Generate a set of points that are an equal distance apart
	// 
	double diststep = 1.0;
	QVector<Pose2d> equidist = makeEqualDistance(discrete, diststep);

	//
	// Step 4: Generate a timing view that meets the constraints of the system
	// 
	std::shared_ptr<ArmMotionProfile> profile = generateTimedProfile(equidist);

	return profile;
}
