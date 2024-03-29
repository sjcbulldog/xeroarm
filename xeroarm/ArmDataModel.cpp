#include "ArmDataModel.h"
#include "JsonFileKeywords.h"
#include "ArmMotionProfileGenerator.h"
#include <QtCore/QFile>
#include <QtCore/QTextStream>

ArmDataModel::ArmDataModel()
{
	clear();

	JointDataModel model(30.0, 0.0);
	addJointModel(model);
	addJointModel(model);
	addJointModel(model);
	dirty_ = false;

	running_ = true;
	generate_ = std::thread(&ArmDataModel::threadFunction, this);
}

ArmDataModel::~ArmDataModel()
{
	running_ = false;
	generate_.join();
}

void ArmDataModel::writeTrajectory(std::shared_ptr<ArmMotionProfile> profile, const QString& filename)
{
	QFile file(filename);
	file.open(QIODevice::WriteOnly);
	QTextStream strm(&file);

	QVector<Pose2dTrajectory> traj = profile->trajectory();
	QString line;
	Pose2dTrajectory prev = traj[0];
	QVector<double> prevvel(2);

	line = "time";
	line += ",position";
	line += ",X";
	line += ",Y";
	line += ",a0";
	line += ",a1";
	line += ",v0";
	line += ",v1";
	line += ",aa0";
	line += ",aa1";
	strm << line << '\n';

	for (int i = 0; i < traj.count(); i++) {
		const Pose2dTrajectory& p = traj[i];
		line = QString::number(p.time());
		line += "," + QString::number(p.position());
		line += "," + QString::number(p.getTranslation().getX());
		line += "," + QString::number(p.getTranslation().getY());
		line += "," + QString::number(p.angles().at(0));
		line += "," + QString::number(p.angles().at(1));

		if (i == 0) {
			line += ",0.0";
			line += ",0.0";
			line += ",0.0";
			line += ",0.0";
		}
		else {
			double deltat = p.time() - prev.time();
			double v0 = (p.angles().at(0) - prev.angles().at(0)) / deltat;
			line += "," + QString::number(v0);
			double v1 = (p.angles().at(1) - prev.angles().at(1)) / deltat;
			line += "," + QString::number(v1);

			double a0 = (v0 - prevvel[0]) / deltat;
			double a1 = (v1 - prevvel[1]) / deltat;

			line += "," + QString::number(a0);
			line += "," + QString::number(a1);

			prevvel[0] = v0;
			prevvel[1] = v1;

		}

		prev = p;
		strm << line << '\n';
	}

	file.close();
}

void ArmDataModel::generateTrajectories()
{
	bool ok = true;

	for (const JointDataModel &joint : joints()) {
		if (joint.maxAccel() <= 0.0 || joint.maxVelocity() <= 0.0) {
			ok = false;
		}
	}

	if (ok) {
		std::lock_guard guard(queue_lock_);
		queue_.clear();
		for (auto path : paths_.values()) {
			queue_.push_back(path);
		}
	}
}

void ArmDataModel::threadFunction()
{
	while (running_)
	{
		std::shared_ptr<ArmPath> path;
		{
			std::lock_guard guard(queue_lock_);
			if (!queue_.isEmpty()) {
				path = queue_.front();
				queue_.pop_front();
			}
		}

		if (path != nullptr) {
			emit progress("Generating data for path '" + path->name() + "'");
			ArmMotionProfileGenerator gen(*this);
			std::shared_ptr<ArmMotionProfile> profile = gen.generateProfile(path);
			path->setProfile(profile);
		}

		if (!running_)
			break;

		bool sleep = false;
		{
			std::lock_guard guard(queue_lock_);
			if (queue_.isEmpty()) {
				emit progress("Idle");
				sleep = true;
			}
		}

		if (sleep) {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}
}

void ArmDataModel::somethingChanged(ChangeType type)
{
	dirty_ = true;
	emit dataChanged(type);
}

bool ArmDataModel::parsePose(const QJsonObject& pos, const QString& name, QString& error, Pose2d& posv)
{
	if (!pos.contains(JsonFileKeywords::XKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::XKeyword + "' member";
		return false;
	}

	if (!pos.value(JsonFileKeywords::XKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::XKeyword + "' but it is not a double value";
		return false;
	}
	double x = pos.value(JsonFileKeywords::XKeyword).toDouble();

	if (!pos.contains(JsonFileKeywords::XKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::XKeyword + "' member";
		return false;
	}

	if (!pos.value(JsonFileKeywords::XKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::XKeyword + "' but it is not a double value";
		return false;
	}
	double y = pos.value(JsonFileKeywords::YKeyword).toDouble();

	double heading = 0.0;
	if (!pos.contains(JsonFileKeywords::HeadingKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::HeadingKeyword + "' member";
		return false;
	}

	if (!pos.value(JsonFileKeywords::HeadingKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::HeadingKeyword + "' but it is not a double value";
		return false;
	}
	
	heading = pos.value(JsonFileKeywords::HeadingKeyword).toDouble();

	posv = Pose2d(x, y, Rotation2d::fromDegrees(heading));
	return true;
}

bool ArmDataModel::parseNamedPosition(const QJsonObject& obj, const QString& name, QString& error, Translation2d& posv)
{
	QString elem;

	if (name.contains('/')) {
		int index = name.lastIndexOf('/');
		elem = name.mid(index);
	}
	else {
		elem = name;
	}

	if (!obj.contains(elem)) {
		error = "json file does not contains '" + name + "' member";
		return false;
	}

	if (!obj.value(elem).isObject()) {
		error = "json file contains member '" + name + "', but it is not an object";
		return false;
	}

	return parsePosition(obj.value(elem).toObject(), name, error, posv);
}

bool ArmDataModel::parsePosition(const QJsonObject& pos, const QString& name, QString& error, Translation2d& posv)
{
	if (!pos.contains(JsonFileKeywords::XKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::XKeyword + "' member";
		return false;
	}

	if (!pos.value(JsonFileKeywords::XKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::XKeyword + "' but it is not a double value";
		return false;
	}
	double x = pos.value(JsonFileKeywords::XKeyword).toDouble();

	if (!pos.contains(JsonFileKeywords::XKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::XKeyword + "' member";
		return false;
	}

	if (!pos.value(JsonFileKeywords::XKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::XKeyword + "' but it is not a double value";
		return false;
	}
	double y = pos.value(JsonFileKeywords::YKeyword).toDouble();

	posv = Translation2d(x, y);
	return true;
}

bool ArmDataModel::parseNamedSize(const QJsonObject& obj, const QString& name, QString& error, Translation2d& szval)
{
	QString elem;

	if (name.contains('/')) {
		int index = name.lastIndexOf('/');
		elem = name.mid(index);
	}
	else {
		elem = name;
	}

	if (!obj.contains(elem)) {
		error = "json file does not contains '" + name + "' member";
		return false;
	}

	if (!obj.value(elem).isObject()) {
		error = "json file contains member '" + name + "', but it is not an object";
		return false;
	}

	return parseSize(obj.value(elem).toObject(), name, error, szval);
}

bool ArmDataModel::parseSize(const QJsonObject& sz, const QString& name, QString& error, Translation2d& szval)
{
	if (!sz.contains(JsonFileKeywords::WidthKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::WidthKeyword + "' member";
		return false;
	}

	if (!sz.value(JsonFileKeywords::WidthKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::WidthKeyword + "' but it is not a double value";
		return false;
	}
	double width = sz.value(JsonFileKeywords::WidthKeyword).toDouble();

	if (!sz.contains(JsonFileKeywords::HeightKeyword)) {
		error = "json file does not contains '" + name + JsonFileKeywords::HeightKeyword + "' member";
		return false;
	}

	if (!sz.value(JsonFileKeywords::HeightKeyword).isDouble()) {
		error = "json file contains member '" + name + JsonFileKeywords::HeightKeyword + "' but it is not a double value";
		return false;
	}
	double height = sz.value(JsonFileKeywords::HeightKeyword).toDouble();

	szval = Translation2d(width, height);
	return true;
}

bool ArmDataModel::parseRobot(const QJsonObject& obj, QString& error)
{
	if (!obj.contains(JsonFileKeywords::RobotKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::RobotKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::RobotKeyword).isObject()) {
		error = "json file contains member '" + QString(JsonFileKeywords::RobotKeyword) + "', but it is not a JSON object";
		return false;
	}

	QJsonObject robot = obj.value(JsonFileKeywords::RobotKeyword).toObject();

	Translation2d temp;
	if (!parseNamedPosition(robot, JsonFileKeywords::ArmPosKeyword, error, temp))
		return false;
	arm_.setPos(temp);

	if (!parseNamedPosition(robot, JsonFileKeywords::BumperPosKeyword, error, bumper_pos_))
		return false;

	if (!parseNamedSize(robot, JsonFileKeywords::BumperSizeKeyword, error, bumper_size_))
		return false;

	return true;
}

bool ArmDataModel::parsePaths(const QJsonObject& obj, QString& error)
{
	if (!obj.contains(JsonFileKeywords::PathsKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::PathsKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::PathsKeyword).isArray()) {
		error = "json file contains member '" + QString(JsonFileKeywords::PathsKeyword) + "', but it is not a JSON array";
		return false;
	}

	QJsonArray jarray = obj.value(JsonFileKeywords::PathsKeyword).toArray();

	for (int i = 0; i < jarray.count(); i++) {
		if (!jarray.at(i).isObject()) {
			error = "json file member '" + QString(JsonFileKeywords::PathsKeyword) + "', entry " + QString::number(i + 1) + " is not a JSON object";
			return false;
		}

		std::shared_ptr<ArmPath> path = std::make_shared<ArmPath>();
		if (!path->fromJson(jarray.at(i).toObject(), error))
			return false;

		addPath(path);
	}

	return true;
}

bool ArmDataModel::parseJoints(const QJsonObject& obj, QString& error)
{
	if (!obj.contains(JsonFileKeywords::JointsKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::JointsKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::JointsKeyword).isArray()) {
		error = "json file contains member '" + QString(JsonFileKeywords::JointsKeyword) + "', but it is not a JSON array";
		return false;
	}

	QJsonArray jarray = obj.value(JsonFileKeywords::JointsKeyword).toArray();

	for (int i = 0; i < jarray.count(); i++) {
		if (!jarray.at(i).isObject()) {
			error = "json file member '" + QString(JsonFileKeywords::JointsKeyword) + "', entry " + QString::number(i + 1) + " is not a JSON object";
			return false;
		}

		JointDataModel joint;
		if (!joint.fromJson(jarray.at(i).toObject(), error))
			return false;

		addJointModel(joint);
	}

	return true;
}

bool ArmDataModel::parseTargets(const QJsonObject& obj, QString& error)
{
	if (!obj.contains(JsonFileKeywords::TargetsKeyword)) {
		error = "json file does not contains '" + QString(JsonFileKeywords::TargetsKeyword) + "' member";
		return false;
	}

	if (!obj.value(JsonFileKeywords::TargetsKeyword).isArray()) {
		error = "json file contains member '" + QString(JsonFileKeywords::TargetsKeyword) + "', but it is not a JSON array";
		return false;
	}

	QJsonArray jarray = obj.value(JsonFileKeywords::TargetsKeyword).toArray();

	for (int i = 0; i < jarray.count(); i++) {
		if (!jarray.at(i).isObject()) {
			error = "json file member '" + QString(JsonFileKeywords::TargetsKeyword) + "', entry " + QString::number(i + 1) + " is not a JSON object";
			return false;
		}

		Translation2d pt;
		QString name = QString(JsonFileKeywords::TargetsKeyword) + " - entry " + QString::number(i);
		if (!parsePosition(jarray.at(i).toObject(), name, error, pt))
			return false;

		addTarget(pt);
	}

	return true;
}

bool ArmDataModel::load(const QString& path, QString& error)
{
	QFile file(path);
	if (!file.open(QIODevice::OpenModeFlag::ReadOnly)) {
		error = file.errorString();
		return false;
	}

	QByteArray text = file.readAll();
	QJsonDocument doc = QJsonDocument::fromJson(text);
	if (doc.isNull())
	{
		error = "cannot parse file '" + file.fileName() + "' for reading";
		return false;
	}

	if (!doc.isObject())
	{
		error = "JSON file '" + file.fileName() + "' does not hold a JSON object";
		return false;
	}

	QJsonObject obj = doc.object();

	clear();

	if (!parseRobot(obj, error))
		return false;

	if (!parseJoints(obj, error))
		return false;

	if (!parsePaths(obj, error))
		return false;

	if (!parseTargets(obj, error))
		return false;

	setToInitialArmPos();
	dirty_ = false;

	generateTrajectories();

	return true;
}

QJsonArray ArmDataModel::targetsToJson()
{
	QJsonArray ret;

	for (int i = 0; i < targets_.size(); i++) {
		QJsonObject target;

		target["x"] = targets_[i].getX();
		target["y"] = targets_[i].getY();
		ret.push_back(target);
	}

	return ret;
}

QJsonArray ArmDataModel::jointsToJson()
{
	QJsonArray ret;

	for (int i = 0; i < arm_.count(); i++) {
		QJsonObject obj = arm_.at(i).toJson();
		ret.push_back(obj);
	}

	return ret;
}

QJsonArray ArmDataModel::pathsToJson()
{
	QJsonArray ret;

	auto values = paths_.values();
	for (int i = 0; i < values.size(); i++) {
		QJsonObject obj = values[i]->toJson();
		ret.push_back(obj);
	}

	return ret;
}

bool ArmDataModel::save(const QString& path, QString& error)
{
	QJsonObject obj;
	QJsonObject robot, pt, sz;
	QJsonObject target;
	QJsonArray arms;

	pt[JsonFileKeywords::XKeyword] = arm_.pos().getX();
	pt[JsonFileKeywords::YKeyword] = arm_.pos().getY();
	robot[JsonFileKeywords::ArmPosKeyword] = pt;

	pt[JsonFileKeywords::XKeyword] = bumper_pos_.getX();
	pt[JsonFileKeywords::YKeyword] = bumper_pos_.getY();
	robot[JsonFileKeywords::BumperPosKeyword] = pt;

	sz[JsonFileKeywords::WidthKeyword] = bumper_size_.getX();
	sz[JsonFileKeywords::HeightKeyword] = bumper_size_.getY();
	robot[JsonFileKeywords::BumperSizeKeyword] = sz;

	obj[JsonFileKeywords::RobotKeyword] = robot;
	obj[JsonFileKeywords::JointsKeyword] = jointsToJson();
	obj[JsonFileKeywords::PathsKeyword] = pathsToJson();
	obj[JsonFileKeywords::TargetsKeyword] = targetsToJson();

	QJsonDocument doc(obj);
	QFile file(path);
	if (!file.open(QIODevice::OpenModeFlag::Truncate | QIODevice::OpenModeFlag::WriteOnly)) {
		error = file.errorString();
		return false;
	}

	file.write(doc.toJson());
	file.close();

	dirty_ = false;

	return true;
}
