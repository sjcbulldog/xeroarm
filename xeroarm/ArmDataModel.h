#pragma once

#include "JointDataModel.h"
#include "ArmPath.h"
#include "MathUtils.h"
#include "Pose2d.h"
#include <QtCore/QPointF>
#include <QtCore/QSizeF>
#include <QtCore/QString>
#include <QtCore/QMap>
#include <QtCore/QJSonDocument>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QList>
#include <memory>

class ArmDataModel : public QObject
{
	Q_OBJECT

public:
	ArmDataModel();

	QPointF getInitialArmPos();
	void setToInitialArmPos();

	bool isDirty() const {
		return dirty_;
	}

	void clear() {
		arm_pos_ = QPointF(0.0, 2.0);
		bumper_pos_ = QPointF(10.5, 2.0);
		bumper_size_ = QSizeF(3.5, 5.0);

		joints_.clear();
		paths_.clear();
		targets_.clear();

		dirty_ = false;
	}

	bool load(const QString& path, QString& error);
	bool save(const QString& path, QString& error);

	void setArmPos(const QPointF& pos) {
		if (MathUtils::epsilonEqual(pos.x(), arm_pos_.x()) == false || MathUtils::epsilonEqual(pos.y(), arm_pos_.y()) == false) {
			arm_pos_ = pos;
			somethingChanged();
		}
	}

	const QPointF& armPos() const {
		return arm_pos_;
	}

	void setBumperPos(const QPointF& pos) {
		if (MathUtils::epsilonEqual(pos.x(), bumper_pos_.x()) == false || MathUtils::epsilonEqual(pos.y(), bumper_pos_.y()) == false) {
			bumper_pos_ = pos;
			somethingChanged();
		}
	}

	const QPointF& bumperPos() const {
		return bumper_pos_;
	}

	void setBumperSize(const QSizeF& sz) {
		if (MathUtils::epsilonEqual(sz.width(), bumper_size_.width()) == false || MathUtils::epsilonEqual(sz.height(), bumper_size_.height()) == false) {
			bumper_size_ = sz;
			somethingChanged();
		}
	}

	const QSizeF& bumperSize() const {
		return bumper_size_;
	}

	const QVector<QPointF>& targets() const {
		return targets_;
	}

	void clearTargets() {
		targets_.clear();
		somethingChanged();
	}

	void addTarget(const QPointF& t) {
		targets_.push_back(t);
		somethingChanged();
	}
	
	void removeTarget(int index) {
		targets_.removeAt(index);
		somethingChanged();
	}

	int jointCount() const {
		return joints_.count();
	}

	const JointDataModel& jointModel(int which) const {
		return joints_[which];
	}

	void addJointModel(const JointDataModel& model) {
		joints_.push_back(model);
		somethingChanged();
	}

	void replaceJointModel(int which, const JointDataModel& model) {
		joints_[which] = model;
		somethingChanged();
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
		somethingChanged();
	}

	void removePath(const QString& name) {
		paths_.remove(name);
		somethingChanged();
	}

	void removePath(std::shared_ptr<ArmPath> path) {
		paths_.remove(path->name());
		somethingChanged();
	}

	void updatePath(std::shared_ptr<ArmPath> path) {
		paths_.insert(path->name(), path);
		somethingChanged();
	}

	void renamePath(const QString& oldname, const QString& newname) {
		auto path = getPathByName(oldname);
		assert(path != nullptr);

		removePath(oldname);
		path->setName(newname);
		addPath(path);
	}

	QList<std::shared_ptr<ArmPath>> getPaths() {
		return paths_.values();
	}

	static bool parseNamedPosition(const QJsonObject& obj, const QString& name, QString& err, QPointF& pos);
	static bool parseNamedSize(const QJsonObject& obj, const QString& name, QString& err, QSizeF& sz);

	static bool parsePosition(const QJsonObject& obj, const QString& name, QString& err, QPointF& pos);
	static bool parsePose(const QJsonObject& obj, const QString& name, QString& err, Pose2d& pos);
	static bool parseSize(const QJsonObject& obj, const QString& name, QString& err, QSizeF& sz);

signals:
	void dataChanged();

private:
	void somethingChanged();

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

	//
	// The position of the ARM relative to the origin of the coordainte system
	//
	QPointF arm_pos_;

	//
	// The position fo the bumpers reltive to the origin of the coordinate system
	//
	QPointF bumper_pos_;

	//
	// The height and width of the bumpers
	//
	QSizeF bumper_size_;

	//
	// The joints that make up the ARM
	//
	QVector<JointDataModel> joints_;

	//
	// The arms paths
	//
	QMap <QString, std::shared_ptr<ArmPath>> paths_;

	//
	// The set of targets
	//
	QVector<QPointF> targets_;
};
