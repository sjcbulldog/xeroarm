#pragma once

#include "InverseKinematics.h"
#include <QtCore/QVector>
#include <Eigen/Dense>

class RobotArm;

class JacobianIK : public InverseKinematics
{
public:
	JacobianIK(const RobotArm& arm);

	QVector<double> inverseKinematics(const Translation2d& pt) const;

private:
	Eigen::MatrixXd computeJacobian(const QVector<double>& angles) const;

private:
	static constexpr const double deltaTheta = 2.0;
	static constexpr const double arrivedThreshold = 0.1;

private:
	const RobotArm& arm_;
};

