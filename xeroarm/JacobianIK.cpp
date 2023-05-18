#include "JacobianIK.h"
#include "Translation2d.h"
#include "RobotArm.h"
#include "MathUtils.h"

using Eigen::MatrixXd;

JacobianIK::JacobianIK(const RobotArm& arm) : arm_(arm)
{
}

MatrixXd JacobianIK::computeJacobian(const QVector<double>& angles) const
{
	MatrixXd ret(2, angles.count());

	//
	// Perform forward kinematics, translate result back to an origin at the
	// base of joint 0.
	//
	Translation2d here = arm_.forwardKinematics(angles).translateBy(arm_.pos().inverse());

	for (int i = 0; i < arm_.count(); i++) {
		QVector<double> deltaangles = angles;

		deltaangles[i] += deltaTheta;

		Translation2d newhere = arm_.forwardKinematics(deltaangles).translateBy(arm_.pos().inverse());
		ret(0, i) = (newhere.getX() - here.getX());
		ret(1, i) = (newhere.getY() - here.getY());
	}

	return ret;
}

QVector<double> JacobianIK::inverseKinematics(const Translation2d& pt) const
{
	const double alpha = 1.0;
	int iters = 0;

	QVector<double> current(arm_.count());
	for (int i = 0; i < arm_.count(); i++) {
		current[i] = arm_.at(i).initialAngle();
	}
	Translation2d curpos = arm_.forwardKinematics(current);

	while (curpos.distance(pt) > arrivedThreshold)
	{
		MatrixXd jacobian = computeJacobian(current);
		MatrixXd inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();

		MatrixXd target(2, 1);
		target(0, 0) = pt.getX() - curpos.getX();
		target(1, 0) = pt.getY() - curpos.getY();

		MatrixXd dt = inverse * target;
		for (int i = 0; i < arm_.count(); i++) {
			current[i] = MathUtils::boundDegrees(current[i] + dt(i) * alpha);
		}

		curpos = arm_.forwardKinematics(current);

		if (iters > 10000) {
			current.clear();
			return current;
		}

		iters++;
	}

	return current;
}
