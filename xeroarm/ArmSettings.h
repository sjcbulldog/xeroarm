#pragma once

#include "ArmPath.h"
#include "OneArmSettings.h"
#include "RobotSettings.h"
#include "TargetPanel.h"
#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtWidgets/QWidget>
#include <QtWidgets/QBoxLayout>
#include <memory>

class ArmDataModel;

class ArmSettings : public QWidget
{
	Q_OBJECT

public:
	ArmSettings(ArmDataModel &model, QWidget* parent);
	~ArmSettings();

	void clear();

private:
	//
	// Changes coming from the GUI
	//
	void jointSettingChanged(int which, ChangeType type);
	void robotSettingChanged(ChangeType);
	void targetSettingChanged();

	//
	// Changes coming from the model
	//
	void modelDataChanged();

	//
	// Add a new joint
	//
	void addJoint(int n, const JointDataModel &model);

private:
	ArmDataModel& model_;
	QHBoxLayout* layout_;
	QVector<OneArmSettings*> settings_;
	RobotSettings* robot_;
	TargetPanel* targets_;
};

