#include "ArmMotionProfileGenerator.h"
#include "SplinePair.h"
#include "Pose2dConstrained.h"

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

void ArmMotionProfileGenerator::getSegmentArc(std::shared_ptr<SplinePair> pair, QVector<Pose2dTrajectory>& results,
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
		results.push_back(Pose2dTrajectory(model_.jointCount(), pair->evalPose(t1)));
	}
}

QVector<Pose2dTrajectory> ArmMotionProfileGenerator::makeDiscrete(const QVector<std::shared_ptr<SplinePair>>& splines, double maxDx, double maxDy, double maxDTheta)
{
	QVector<Pose2dTrajectory> results;

	results.push_back(Pose2dTrajectory(model_.jointCount(), splines[0]->getStartPose()));
	for (int i = 0; i < splines.size(); i++)
		getSegmentArc(splines[i], results, 0.0, 1.0, maxDx, maxDy, maxDTheta);

	return results;
}

QVector<Pose2dTrajectory> ArmMotionProfileGenerator::makeEqualDistance(const QVector<Pose2dTrajectory>& points, double step)
{
	QVector<double> distances;
	QVector<Pose2dTrajectory> result;

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
		Pose2dTrajectory newpttraj(model_.jointCount(), points[index].interpolate(points[index + 1], percent));

		QVector<double> angles = model_.arm().inverseKinematicsJacobian(newpttraj.getTranslation());
		newpttraj.setAngles(angles);

		result.push_back(newpttraj);
	}

	distances.clear();
	distances.push_back(0.0);
	for (int i = 1; i < result.size(); i++)
		distances.push_back(result[i].distance(result[i - 1]) + distances[i - 1]);

	return result;
}

double ArmMotionProfileGenerator::jointConstrainedVelocity(const Pose2dConstrained& state, const Pose2dConstrained &pred)
{
	QVector<double> angles = model_.arm().inverseKinematicsJacobian(state.pose().getTranslation());
	return std::numeric_limits<double>::max();
}

void ArmMotionProfileGenerator::computeJointValues(Pose2dTrajectory& traj)
{
}

std::shared_ptr<ArmMotionProfile> ArmMotionProfileGenerator::generateTimedProfile(std::shared_ptr<ArmPath> path, const QVector<Pose2dTrajectory>& view)
{
	QVector<Pose2dConstrained> points;
	Pose2dConstrained predecessor(model_.jointCount());
	const static double kEpsilon = 1e-6;
	double maxaccel = 12.0;
	double maxvel = 12.0;

	predecessor.setPosition(0.0);
	predecessor.setPose(view[static_cast<int>(0)]);
	predecessor.setVelocity(0);
	predecessor.setAccelMin(-maxaccel);
	predecessor.setAccelMax(maxaccel);

	//
	// Forward pass
	//
	for (int i = 0; i < view.size(); i++)
	{
		Pose2dConstrained state(model_.jointCount());
		state.setPose(view[i]);

		double dist = predecessor.pose().distance(state.pose());
		state.setPosition(predecessor.position() + dist);

		while (true)
		{
			double calcvel = std::sqrt(predecessor.velocity() * predecessor.velocity() + 2.0 * predecessor.accelMax() * dist);
			state.setVelocity(std::min(maxvel, calcvel));

			if (std::isnan(state.velocity()))
				throw std::runtime_error("invalid maximum velocity");

			state.setAccelMin(-maxaccel);
			state.setAccelMax(maxaccel);

			double convel = jointConstrainedVelocity(state, predecessor);
			state.setVelocity(std::min(state.velocity(), convel));

			if (state.velocity() < 0.0)
				throw std::runtime_error("invalid maximum velocity - constraint set to negative");

			if (dist < kEpsilon)
				break;

			double actaccel = (state.velocity() * state.velocity() - predecessor.velocity() * predecessor.velocity()) / (2.0 * dist);
			if (state.accelMax() < actaccel - kEpsilon)
			{
				predecessor.setAccelMax(state.accelMax());
				if (i != 0)
					points[i - 1] = predecessor;
			}
			else
			{
				if (actaccel > predecessor.accelMin() + kEpsilon)
				{
					predecessor.setAccelMax(actaccel);
					if (i != 0)
						points[i - 1] = predecessor;
				}
				break;
			}
		}
		points.push_back(state);
		predecessor = state;
	}

	//
	// Backward pass
	//
	int last = view.size() - 1;
	Pose2dConstrained sucessor(model_.jointCount());
	sucessor.setPose(view[last]);
	sucessor.setPosition(points[last].position());
	sucessor.setVelocity(0.0);
	sucessor.setAccelMin(-maxaccel);
	sucessor.setAccelMax(maxaccel);

	for (int i = points.size() - 1; i >= 0; i--)
	{
		Pose2dConstrained state = points[i];
		double dist = state.position() - sucessor.position();				// Will be negative

		while (true)
		{
			double tmp = sucessor.velocity() * sucessor.velocity() + 2.0 * sucessor.accelMin() * dist;
			double newmaxvel = std::sqrt(tmp);
			if (newmaxvel >= state.velocity())
				break;

			if (std::isnan(newmaxvel))
				throw std::runtime_error("invalid new maximum velocity");

			state.setVelocity(newmaxvel);
			points[i] = state;

			if (dist > kEpsilon)
				break;

			double actaccel = (state.velocity() * state.velocity() - sucessor.velocity() * sucessor.velocity()) / (2.0 * dist);
			if (state.accelMin() > actaccel + kEpsilon)
			{
				sucessor.setAccelMin(state.accelMin());
				if (i != points.size() - 1)
					points[i + 1] = sucessor;
			}
			else
			{
				sucessor.setAccelMin(actaccel);
				if (i != points.size() - 1)
					points[i + 1] = sucessor;

				break;
			}
		}
		sucessor = state;
	}

	double t = 0.0;
	double s = 0.0;
	double v = 0.0;
	QVector<Pose2dTrajectory> result;

	for (int i = 0; i < points.size(); i++)
	{
		QVector<double> avel, aacel;

		const Pose2dConstrained& state = points[i];
		double ds = state.position() - s;
		double accel = (state.velocity() * state.velocity() - v * v) / (2.0 * ds);
		double dt = 0;
		if (i > 0)
		{
			result[i - 1].setAccel(accel);
			if (std::fabs(accel) > kEpsilon)
			{
				dt = (state.velocity() - v) / accel;
			}
			else
			{
				dt = ds / v;
			}

			for (int j = 0; j < model_.jointCount(); j++) {
				double aveln = (points[i - 1].pose().angles().at(j) - points[i].pose().angles().at(j)) / dt;
				double aaceln = (result[i - 1].velocities().at(j) - aveln) / dt;
				avel.push_back(aveln);
				aacel.push_back(aaceln);
			}
		}
		else {
			avel.resize(model_.jointCount());
			aacel.resize(model_.jointCount());
		}

		t += dt;
		if (std::isinf(t) || std::isnan(t))
			throw std::runtime_error("invalid time value integrating forward");

		v = state.velocity();
		s = state.position();

		Pose2dTrajectory trajpt(view[i], t, s, v, accel);
		trajpt.setVelocities(avel);
		trajpt.setAaccel(aacel);

		result.push_back(trajpt);
	}

	return std::make_shared<ArmMotionProfile>(path, result);
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
	QVector<Pose2dTrajectory> discrete = makeDiscrete(splines, maxDx, maxDy, maxDTheta);

	//
	// Step 3: Generate a set of points that are an equal distance apart
	// 
	double diststep = 1.0;
	QVector<Pose2dTrajectory> equidist = makeEqualDistance(discrete, diststep);

	//
	// Step 4: Generate a timing view that meets the constraints of the system
	// 
	std::shared_ptr<ArmMotionProfile> profile = generateTimedProfile(path, equidist);

	return profile;
}
