#pragma once

#include "JointDataModel.h"
#include "RobotArm.h"
#include "ArmPath.h"
#include "MathUtils.h"
#include "Pose2d.h"
#include "ChangeType.h"
#include <QtCore/QPointF>
#include <QtCore/QSizeF>
#include <QtCore/QString>
#include <QtCore/QMap>
#include <QtCore/QJSonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QList>
#include <memory>
#include <thread>
#include <mutex>

class ArmMotionProfile;

class ArmDataModel : public QObject
{
	Q_OBJECT

public:
	ArmDataModel();
	virtual ~ArmDataModel();

	void generateTrajectories();

	Translation2d getInitialArmPos() {
		return arm_.getInitialArmPos();
	}

	void setToInitialArmPos() {
		arm_.setToInitialArmPos();
	}

	bool isDirty() const {
		return dirty_;
	}

	void clearDirtyFlag() {
		dirty_ = false;
	}

	void writeTrajectory(std::shared_ptr<ArmMotionProfile> profile, const QString &filename);

	void clear() {
		arm_.setPos(Translation2d(0.0, 2.0));
		bumper_pos_ = Translation2d(10.5, 2.0);
		bumper_size_ = Translation2d(3.5, 5.0);

		arm_.clear();
		paths_.clear();
		targets_.clear();

		dirty_ = false;
	}

	bool load(const QString& path, QString& error);
	bool save(const QString& path, QString& error);

	void setArmPos(const Translation2d& pos) {
		if (MathUtils::epsilonEqual(pos.getX(), arm_.pos().getX()) == false || MathUtils::epsilonEqual(pos.getY(), arm_.pos().getY()) == false) {
			arm_.setPos(pos);
			somethingChanged(ChangeType::ArmPos);
		}
	}

	const Translation2d& armPos() const {
		return arm_.pos();
	}

	void setBumperPos(const Translation2d& pos) {
		if (MathUtils::epsilonEqual(pos.getX(), bumper_pos_.getX()) == false || MathUtils::epsilonEqual(pos.getY(), bumper_pos_.getY()) == false) {
			bumper_pos_ = pos;
			somethingChanged(ChangeType::BumperPos);
		}
	}

	const Translation2d& bumperPos() const {
		return bumper_pos_;
	}

	void setBumperSize(const Translation2d& sz) {
		if (MathUtils::epsilonEqual(sz.getX(), bumper_size_.getX()) == false || MathUtils::epsilonEqual(sz.getY(), bumper_size_.getY()) == false) {
			bumper_size_ = sz;
			somethingChanged(ChangeType::BumperSize);
		}
	}

	const Translation2d& bumperSize() const {
		return bumper_size_;
	}

	const QVector<Translation2d>& targets() const {
		return targets_;
	}

	void clearTargets() {
		targets_.clear();
		somethingChanged(ChangeType::Targets);
	}

	void addTarget(const Translation2d& t) {
		targets_.push_back(t);
		somethingChanged(ChangeType::Targets);
	}
	
	void removeTarget(int index) {
		targets_.removeAt(index);
		somethingChanged(ChangeType::Targets);
	}

	QVector<JointDataModel>& joints() {
		return arm_.joints();
	}

	int jointCount() const {
		return arm_.count();
	}

	const JointDataModel& jointModel(int which) const {
		return arm_.at(which);
	}

	void addJointModel(const JointDataModel& model) {
		arm_.addJoint(model);
		somethingChanged(ChangeType::AddJoint);
	}

	void replaceJointModel(int which, const JointDataModel& model) {
		arm_.replaceJoint(which, model);
		somethingChanged(ChangeType::UpdateJoint);
	}

	void setJointAngle(int which, double angle) {
		arm_.setJointAngle(which, angle);
		somethingChanged(ChangeType::UpdateJoint);
	}

	RobotArm& arm() {
		return arm_;
	}


	bool hasPath(const QString& name) const {
		return paths_.contains(name);
	}

	std::shared_ptr<ArmPath> getPathByName(const QString& name) const {
		if (!paths_.contains(name))
			return nullptr;

		return paths_.value(name);
	}

	void addPath(std::shared_ptr<ArmPath> path) {
		paths_.insert(path->name(), path);
		somethingChanged(ChangeType::AddPath);
	}

	void removePath(const QString& name) {
		paths_.remove(name);
		somethingChanged(ChangeType::RemovePath);
	}

	void removePath(std::shared_ptr<ArmPath> path) {
		paths_.remove(path->name());
	}

	void renamePath(const QString& oldname, const QString& newname) {
		auto path = getPathByName(oldname);
		assert(path != nullptr);

		removePath(oldname);
		path->setName(newname);
		addPath(path);

		somethingChanged(ChangeType::RenamePath);
	}

	QList<std::shared_ptr<ArmPath>> getPaths() {
		return paths_.values();
	}

	void pathPointChanged() {
		dirty_ = true;
		emit dataChanged(ChangeType::PathPoint);
	}

	static bool parseNamedPosition(const QJsonObject& obj, const QString& name, QString& err, Translation2d& pos);
	static bool parseNamedSize(const QJsonObject& obj, const QString& name, QString& err, Translation2d& sz);

	static bool parsePosition(const QJsonObject& obj, const QString& name, QString& err, Translation2d& pos);
	static bool parsePose(const QJsonObject& obj, const QString& name, QString& err, Pose2d& pos);
	static bool parseSize(const QJsonObject& obj, const QString& name, QString& err, Translation2d& sz);

signals:
	void dataChanged(ChangeType type);
	void progress(const QString& msg);

private:
	void somethingChanged(ChangeType type);
	void threadFunction();

	QJsonArray targetsToJson();
	QJsonArray jointsToJson();
	QJsonArray pathsToJson();

	bool parseRobot(const QJsonObject& obj, QString &err);
	bool parseTargets(const QJsonObject& obj, QString& err);
	bool parseJoints(const QJsonObject& obj, QString& err);
	bool parsePaths(const QJsonObject& obj, QString& err);


private:
	//
	// If true, the model has been changed since it was last saved 
	//
	bool dirty_;

	bool running_;

	RobotArm arm_;

	//
	// The position fo the bumpers reltive to the origin of the coordinate system
	//
	Translation2d bumper_pos_;

	//
	// The height and width of the bumpers
	//
	Translation2d bumper_size_;

	//
	// The arms paths
	//
	QMap<QString, std::shared_ptr<ArmPath>> paths_;

	//
	// The motion profiles for each path
	//
	QMap<QString, std::shared_ptr<ArmMotionProfile>> profiles_;

	//
	// The set of targets
	//
	QVector<Translation2d> targets_;

	std::mutex queue_lock_;
	std::thread generate_;
	QVector<std::shared_ptr<ArmPath>> queue_;
};
