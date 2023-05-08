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
	}

	JointDataModel(double length, double init) {
		length_ = length;
		angle_ = init;
		initial_angle_ = init;
		maxv_ = 0.0;
		maxa_ = 0.0;
	}

	bool isAngleValid(double a) const {
		for (const QPair<double, double>& pair : keep_out_regions_) {
			if (a >= pair.first && a <= pair.second)
				return false;
		}

		return true;
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
	// A set of rotation values that define regions where a joint cannot
	// be.
	//
	QVector<QPair<double, double>> keep_out_regions_;
};

