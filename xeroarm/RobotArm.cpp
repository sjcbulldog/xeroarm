#include "RobotArm.h"
#include "JacobianIK.h"
#include <Eigen/Dense>
#include <Eigen/QR>

RobotArm::RobotArm()
{
	inverse_ = new JacobianIK(*this);
}

RobotArm::~RobotArm()
{
	delete inverse_;
}

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

Translation2d RobotArm::jointStartPos(int joint, const QVector<double>& angles) const
{
	Translation2d endpos = pos_;
	double baseangle = 0.0;

	for (int i = 0; i < joint; i++) {
		double angle = baseangle + angles[i] * M_PI / 180.0;
		double length = joints_.at(i).length();

		endpos = Translation2d(endpos.getX() + std::cos(angle) * length, endpos.getY() + std::sin(angle) * length);
		baseangle = angle;
	}

	return endpos;
}

Translation2d RobotArm::forwardKinematics(const QVector<double>& angles) const
{
	Translation2d endpos = pos_;
	double baseangle = 0.0;

	for (int i = 0; i < angles.count(); i++) {
		double angle = baseangle + angles[i] * M_PI / 180.0;
		double length = joints_.at(i).length();

		endpos = Translation2d(endpos.getX() + std::cos(angle) * length, endpos.getY() + std::sin(angle) * length);
		baseangle = angle;
	}

	return endpos;
}

