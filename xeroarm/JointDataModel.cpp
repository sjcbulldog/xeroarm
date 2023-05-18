#include "JointDataModel.h"
#include "JsonFileKeywords.h"
#include <QtCore/QJsonArray>

QJsonObject JointDataModel::toJson() const
{
	QJsonObject obj;
	QJsonArray jarray;

	obj[JsonFileKeywords::LengthKeyword] = length_;
	obj[JsonFileKeywords::InitialAngleKeyword] = initial_angle_;
	obj[JsonFileKeywords::MaxVelocityKeyword] = maxv_;
	obj[JsonFileKeywords::MaxAccelKeyword] = maxa_;
	return obj;
}

bool JointDataModel::fromJson(const QJsonObject& obj, QString& error)
{
	if (!obj.contains(JsonFileKeywords::LengthKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::LengthKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::LengthKeyword).isDouble()) {
		error = "json file contains member '" + QString(JsonFileKeywords::LengthKeyword) + "', but it is not a double";
		return false;
	}

	length_ = obj.value(JsonFileKeywords::LengthKeyword).toDouble();

	if (!obj.contains(JsonFileKeywords::InitialAngleKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::InitialAngleKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::InitialAngleKeyword).isDouble()) {
		error = "json file contains member '" + QString(JsonFileKeywords::InitialAngleKeyword) + "', but it is not a double";
		return false;
	}

	initial_angle_ = obj.value(JsonFileKeywords::InitialAngleKeyword).toDouble();

	if (!obj.contains(JsonFileKeywords::MaxVelocityKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::MaxVelocityKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::MaxVelocityKeyword).isDouble()) {
		error = "json file contains member '" + QString(JsonFileKeywords::MaxVelocityKeyword) + "', but it is not a double";
		return false;
	}

	maxv_ = obj.value(JsonFileKeywords::MaxVelocityKeyword).toDouble();

	if (!obj.contains(JsonFileKeywords::MaxAccelKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::MaxAccelKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::MaxAccelKeyword).isDouble()) {
		error = "json file contains member '" + QString(JsonFileKeywords::MaxAccelKeyword) + "', but it is not a double";
		return false;
	}

	maxa_ = obj.value(JsonFileKeywords::MaxAccelKeyword).toDouble();

	return true;
}