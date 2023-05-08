#include "RobotArm.h"

void RobotArm::setToInitialArmPos()
{
	for (int i = 0; i < joints_.count(); i++) {
		JointDataModel& model = joints_[i];
		model.setAngle(model.initialAngle());
	}
}

Translation2d RobotArm::getInitialArmPos()
{
	QVector<double> angles;
	for (int i = 0; i < joints_.count(); i++) {
		angles.push_back(joints_.at(i).initialAngle());
	}

	return forwardKinematics(angles);
}

double RobotArm::maxArmLength() const 
{
	auto accum = [](double a, const JointDataModel& j) {
		return a + j.length();
	};

	return std::accumulate(joints().begin(), joints().end(), 0.0, accum);
}

Translation2d RobotArm::forwardKinematics(const QVector<double>& angles) const
{
	Translation2d endpos = pos_;
	double baseangle = 0.0;

	for (int i = 0; i < joints_.count(); i++) {
		double angle = baseangle + angles[i] * M_PI / 180.0;
		double length = joints_.at(i).length();

		endpos = Translation2d(endpos.getX() + std::cos(angle) * length, endpos.getY() + std::sin(angle) * length);
		baseangle = angle;
	}

	return endpos;
}

QVector<double> RobotArm::inverseKinematics(const Translation2d& pt, bool initangle) const
{
	QVector<double> current(joints_.count());

	for (int i = 0; i < joints_.count(); i++) {
		current[i] = joints_.at(i).initialAngle();
	}

	Translation2d here = forwardKinematics(current);
	double eps = 0.5;
	double delta = 0.1;
	double error;

	int iter = 1;
	while (true) {
		error = pt.distance(here);
		if (error < eps)
		{
			break; 
		}

		for (int i = current.count() - 1; i >= 0; i--) {
			double save = current[i];
			Translation2d newhere;
			double dist1, dist2;

			current[i] = save + delta;
			if (joints_.at(i).isAngleValid(current[i])) {
				newhere = forwardKinematics(current);
				dist1 = pt.distance(newhere);
			}
			else {
				dist1 = std::numeric_limits<double>::max();
			}

			current[i] = save - delta;
			if (joints_.at(i).isAngleValid(current[i])) {
				newhere = forwardKinematics(current);
				dist2 = pt.distance(newhere);
			}
			else {
				dist2 = std::numeric_limits<double>::max();
			}

			if (dist1 < error && dist1 < dist2) {
				current[i] = save + delta;
			}
			else if (dist2 < error && dist2 < dist2) {
				current[i] = save - delta;
			}
		}

		here = forwardKinematics(current);
		QString anglestr;
		for (int i = 0; i < current.count(); i++) {
			if (anglestr.length() > 0) {
				anglestr += ", ";
			}
			anglestr += QString::number(current[i], 'f', 2);
		}

		iter++;

		if (iter > 25000) {
			current.clear();
			break;
		}
	}

	return current;
}