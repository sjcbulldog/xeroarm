#pragma once

#include <QtCore/QVector>
#include <QtCore/QPair>
#include <QtCore/QJsonObject>

class JointDataModel
{
public:
	JointDataModel() {
		angle_ = 0.0;
		length_ = 0.0;
		initial_angle_ = 0.0;
		maxa_ = 0.0;
		maxv_ = 0.0;
		cw_constraint_ = 0.0;
		ccw_constraint_ = 0.0;
	}

	JointDataModel(double length, double init) {
		length_ = length;
		angle_ = init;
		initial_angle_ = init;
		maxv_ = 0.0;
		maxa_ = 0.0;
		cw_constraint_ = 0.0;
		ccw_constraint_ = 0.0;
	}

	double angle() const {
		return angle_;
	}

	void setAngle(double d) {
		angle_ = d;
	}

	double length() const {
		return length_;
	}

	void setLength(double d) {
		length_ = d;
	}

	double initialAngle() const {
		return initial_angle_;
	}

	void setinitialAngle(double d) {
		initial_angle_ = d;
	}

	double maxVelocity() const {
		return maxv_;
	}

	void setMaxVelocity(double d) {
		maxv_ = d;
	}

	double maxAccel() const {
		return maxv_;
	}

	void setMaxAccel(double d) {
		maxv_ = d;
	}

	QJsonObject toJson() const;
	bool fromJson(const QJsonObject& obj, QString& error);

private:
	//
	// The current angle for the joint
	//
	double angle_;

	//
	// The length of the arm
	//
	double length_;

	//
	// The initial angle (in degrees) for the ARM when at rest
	//
	double initial_angle_;

	//
	// Max velocity of this joint in degrees/second
	//
	double maxv_;

	//
	// Max acceleration of this joing in degrees/second/second
	//
	double maxa_;

	//
	// Constraints for the joint
	//
	double cw_constraint_;
	double ccw_constraint_;
};
