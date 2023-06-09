#pragma once

#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QPointF>
#include <QtCore/QJsonObject>
#include <memory>
#include "ArmMotionProfile.h"
#include "Pose2d.h"

class ArmPath
{
public:
	ArmPath() {
	}

	ArmPath(const QString& name) {
		name_ = name;
	}

	void setName(const QString& name) {
		name_ = name;
	}

	const QString& name() const {
		return name_;
	}

	void addPoint(const Pose2d& pt) {
		points_.push_back(pt);
	}

	void insertPoint(int index, const Pose2d& pt) {
		points_.insert(index + 1, pt);
	}

	int size() const {
		return points_.size();
	}

	int count() const {
		return points_.size();
	}

	const Pose2d& at(int index) {
		return points_[index];
	}

	const Pose2d& operator[](int index) {
		return points_[index];
	}

	void replacePoint(int index, const Pose2d& pt) {
		points_[index] = pt;
	}

	void setProfile(std::shared_ptr<ArmMotionProfile> profile) {
		profile_ = profile;
	}

	std::shared_ptr<ArmMotionProfile> profile() {
		return profile_;
	}

	QJsonObject toJson();
	bool fromJson(const QJsonObject& obj, QString &error);

private:
	QString name_;
	QVector<Pose2d> points_;
	std::shared_ptr<ArmMotionProfile> profile_;
};
