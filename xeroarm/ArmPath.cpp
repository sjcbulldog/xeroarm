#include "ArmPath.h"
#include "JsonFileKeywords.h"
#include "ArmDataModel.h"
#include <QtCore/QJsonArray>

QJsonObject ArmPath::toJson()
{
	QJsonObject obj, pt;
	QJsonArray points;

	obj[JsonFileKeywords::NameKeyword] = name_;

	for (int i = 0; i < points_.count(); i++) {
		pt[JsonFileKeywords::XKeyword] = points_.at(i).getTranslation().getX();
		pt[JsonFileKeywords::YKeyword] = points_.at(i).getTranslation().getY();
		pt[JsonFileKeywords::HeadingKeyword] = points_.at(i).getRotation().toDegrees();
		points.push_back(pt);
	}

	obj[JsonFileKeywords::PointsKeyword] = points;
	return obj;
}

bool ArmPath::fromJson(const QJsonObject& obj, QString& error)
{
	name_.clear();
	points_.clear();

	if (!obj.contains(JsonFileKeywords::NameKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::NameKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::NameKeyword).isString()) {
		error = "json file contains member '" + QString(JsonFileKeywords::NameKeyword) + "', but it is not a string";
		return false;
	}

	name_ = obj.value(JsonFileKeywords::NameKeyword).toString();

	if (!obj.contains(JsonFileKeywords::PointsKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::PointsKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::PointsKeyword).isArray()) {
		error = "json file contains member '" + QString(JsonFileKeywords::PointsKeyword) + "', but it is not an array";
		return false;
	}

	QJsonArray points = obj.value(JsonFileKeywords::PointsKeyword).toArray();

	for (int i = 0; i < points.count(); i++) {
		if (!points.at(i).isObject()) {
			error = "json file member in paths array '" + QString(JsonFileKeywords::PointsKeyword) + "', entry " + QString::number(i + 1) + " is not a JSON object";
			return false;
		}

		Pose2d pt;
		if (!ArmDataModel::parsePose(points.at(i).toObject(), "points", error, pt))
			return false;

		points_.push_back(pt);
	}

	return true;
}