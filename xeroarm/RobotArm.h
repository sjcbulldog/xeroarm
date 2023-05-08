#pragma once

#include "JointDataModel.h"
#include "Translation2d.h"
#include <QtCore/QVector>
#include <QtCore/QPointF>

class RobotArm
{
public:
	RobotArm() {
	}

	virtual ~RobotArm() {
	}

	void setToInitialArmPos();
	Translation2d getInitialArmPos();

	void clear() {
		joints_.clear();
	}

	void setJointAngle(int which, double angle) {
		joints_[which].setAngle(angle);
	}

	void addJoint(const JointDataModel& model) {
		joints_.push_back(model);
	}

	void replaceJoint(int which, const JointDataModel& model) {
		joints_[which] = model;
	}

	const QVector<JointDataModel>& joints() const {
		return joints_;
	}

	int count() const {
		return joints_.count();
	}

	const JointDataModel& at(int which) const {
		return joints_.at(which);
	}

	void setPos(const Translation2d& pos) {
		pos_ = pos;
	}

	const Translation2d& pos() const {
		return pos_;
	}

	double maxArmLength() const;
	QVector<double> inverseKinematics(const Translation2d& pt, bool initangle = true) const;
	Translation2d forwardKinematics(const QVector<double>& angles) const;

private:
	double lawOfCosines(double a, double b, double c) {
		return std::acos((a * a + b * b - c * c) / (2 * a * b));
	}

private:

	Translation2d pos_;

	//
	// The joints that make up the ARM
	//
	QVector<JointDataModel> joints_;
};
