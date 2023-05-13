#include "RobotArm.h"
#include <Eigen/Dense>
#include <Eigen/QR>

using Eigen::MatrixXd;

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

MatrixXd RobotArm::computeJacobian(const QVector<double> &angles) const
{
	MatrixXd ret(2, joints_.count());

	//
	// Perform forward kinematics, translate result back to an origin at the
	// base of joint 0.
	//
	Translation2d here = forwardKinematics(angles).translateBy(pos_.inverse());

	for (int i = 0; i < joints_.count(); i++) {
		QVector<double> deltaangles = angles;
		deltaangles[i] += deltaTheta;

		Translation2d newhere = forwardKinematics(deltaangles).translateBy(pos_.inverse());
		ret(0, i) = (newhere.getX() - here.getX());
		ret(1, i) = (newhere.getY() - here.getY());
	}

	return ret;
}

static QString matrixToString(const MatrixXd& m)
{
	QString out;

	out += "rows " + QString::number(m.rows());
	out += ", cols " + QString::number(m.cols());
	for (int row = 0; row < m.rows(); row++) {
		out += ", row " + QString::number(row) + " ";
		for (int col = 0; col < m.cols(); col++) {
			out += " " + QString::number(m(row, col));
		}
	}

	return out;
}

static QString vectorToString(const QVector<double>& v)
{
	QString out;

	out += "count " + QString::number(v.count());
	out += " data";
	for (int i = 0; i < v.count(); i++) {
		out += " " + QString::number(v.at(i));
	}
	return out;
}

QVector<double> RobotArm::inverseKinematicsJacobian(const Translation2d& pt) const
{
	const double alpha = 0.5;
	int iters = 0;

	QVector<double> current(joints_.count());
	for (int i = 0; i < joints_.count(); i++) {
		current[i] = joints_.at(i).initialAngle();
	}
	Translation2d curpos = forwardKinematics(current).translateBy(pos_.inverse());

	while (curpos.distance(pt) > arrivedThreshold)
	{
		qDebug() << "-----------------------------------------------------------------";
		qDebug() << "iter: " << iters;
		qDebug() << "angles: " << vectorToString(current);
		qDebug() << "target: " << QString::number(pt.getX()) << ", " << QString::number(pt.getY());
		qDebug() << "curpos: " << QString::number(curpos.getX()) << ", " << QString::number(curpos.getY());

		MatrixXd jacobian = computeJacobian(current);
		qDebug() << "IK: jacobian " << matrixToString(jacobian);

		MatrixXd inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
		qDebug() << "IK: inverse " + matrixToString(inverse);

		MatrixXd target(2, 1);
		target(0, 0) = pt.getX() - curpos.getX();
		target(1, 0) = pt.getY() - curpos.getY();

		MatrixXd dt = inverse * target;
		for (int i = 0; i < joints_.count(); i++) {
			current[i] = current[i] + dt(i) * alpha;
		}

		curpos = forwardKinematics(current);
		iters++;

		if (iters > 1000) {
			current.clear();
			return current;
		}
	}

	return current;
}

QVector<double> RobotArm::inverseKinematicsSimple(const Translation2d& pt) const
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