#pragma once

#include "InverseKinematics.h"
#include "JointDataModel.h"
#include "Translation2d.h"
#include <QtCore/QVector>
#include <QtCore/QPointF>


class RobotArm
{
public:

	RobotArm();
	virtual ~RobotArm();

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

	QVector<JointDataModel>& joints() {
		return joints_;
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
	Translation2d forwardKinematics(const QVector<double>& angles) const;
	Translation2d jointStartPos(int joint, const QVector<double>& angles) const;

	QVector<double> angles() const {
		QVector<double> ret;

		for (int i = 0; i < joints_.count(); i++) {
			ret.push_back(joints_.at(i).angle());
		}
		return ret;
	}

	QVector<double> inverseKinematics(const Translation2d& pt) const {
		return inverse_->inverseKinematics(pt);
	}

private:

	//
	// The base position of the ARM
	//
	Translation2d pos_;

	//
	// The joints that make up the ARM
	//
	QVector<JointDataModel> joints_;

	//
	// The inverse kinematics used for the ARM
	//
	InverseKinematics* inverse_;

};
