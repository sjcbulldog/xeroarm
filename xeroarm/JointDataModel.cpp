#include "JointDataModel.h"
#include "JsonFileKeywords.h"
#include <QtCore/QJsonArray>

QJsonObject JointDataModel::toJson()
{
	QJsonObject obj;
	QJsonArray jarray;

	obj[JsonFileKeywords::LengthKeyword] = length_;
	obj[JsonFileKeywords::initialAngleKeyword] = initial_angle_;

	for (int i = 0; i < keep_out_regions_.count() ; i++) {
		QJsonObject keep;
		keep[JsonFileKeywords::MinKeyword] = keep_out_regions_.at(i).first;
		keep[JsonFileKeywords::MaxKeyword] = keep_out_regions_.at(i).second;
		jarray.push_back(keep);
	}

	obj[JsonFileKeywords::KeepOutKeyword] = jarray;

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

	if (!obj.contains(JsonFileKeywords::initialAngleKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::initialAngleKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::initialAngleKeyword).isDouble()) {
		error = "json file contains member '" + QString(JsonFileKeywords::initialAngleKeyword) + "', but it is not a double";
		return false;
	}

	initial_angle_ = obj.value(JsonFileKeywords::initialAngleKeyword).toDouble();

	if (!obj.contains(JsonFileKeywords::KeepOutKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::KeepOutKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::KeepOutKeyword).isArray()) {
		error = "json file contains member '" + QString(JsonFileKeywords::KeepOutKeyword) + "', but it is not an array";
		return false;
	}

	QJsonArray points = obj.value(JsonFileKeywords::KeepOutKeyword).toArray();

	for (int i = 0; i < points.count(); i++) {
		if (!points.at(i).isObject()) {
			error = "json file member in paths array '" + QString(JsonFileKeywords::KeepOutKeyword) + "', entry " + QString::number(i + 1) + " is not a JSON object";
			return false;
		}

		QJsonObject keep = points.at(i).toObject();

		if (!keep.contains(JsonFileKeywords::MinKeyword)) {
			error = "json file does not contains '" + QString(JsonFileKeywords::MinKeyword) + "' member";
			return false;
		}

		if (!keep.value(JsonFileKeywords::MinKeyword).isDouble()) {
			error = "json file contains member '" + QString(JsonFileKeywords::MinKeyword) + "', but it is not a double";
			return false;
		}

		double minv = keep.value(JsonFileKeywords::MinKeyword).toDouble();

		if (!keep.contains(JsonFileKeywords::MaxKeyword)) {
			error = "json file does not contains '" + QString(JsonFileKeywords::MaxKeyword) + "' member";
			return false;
		}

		if (!keep.value(JsonFileKeywords::MaxKeyword).isDouble()) {
			error = "json file contains member '" + QString(JsonFileKeywords::MaxKeyword) + "', but it is not a double";
			return false;
		}

		double maxv = keep.value(JsonFileKeywords::MaxKeyword).toDouble();

		keep_out_regions_.push_back(std::make_pair(minv, maxv));
	}

	return true;
}