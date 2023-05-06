#pragma once

#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QPointF>
#include <QtCore/QJsonObject>
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

	QJsonObject toJson();
	bool fromJson(const QJsonObject& obj, QString &error);

private:
	QString name_;
	QVector<Pose2d> points_;
};
