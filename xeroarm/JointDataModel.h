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
	}

	JointDataModel(double length, double init) {
		length_ = length;
		angle_ = init;
		initial_angle_ = init;
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

	void clearKeepOut() {
		keep_out_regions_.clear();
	}

	void addKeepOut(QPair<double, double> p) {
		keep_out_regions_.push_back(p);
	}

	int keepOutCount() const {
		return keep_out_regions_.count();
	}

	QPair<double, double> keepOut(int which) const {
		return keep_out_regions_.at(which);
	}

	QJsonObject toJson();
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
	// A set of rotation values that define regions where a joint cannot
	// be.
	//
	QVector<QPair<double, double>> keep_out_regions_;
};

