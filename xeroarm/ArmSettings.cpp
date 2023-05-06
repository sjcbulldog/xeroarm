#include "ArmSettings.h"
#include "ArmDataModel.h"
#include <QtWidgets/QBoxLayout>

ArmSettings::ArmSettings(ArmDataModel &model, QWidget* parent) : QWidget(parent), model_(model)
{
	layout_ = new QHBoxLayout();

	robot_ = new RobotSettings(this);
	robot_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	(void)connect(robot_, &RobotSettings::settingsChanged, this, &ArmSettings::robotSettingChanged);
	layout_->addWidget(robot_);
	robot_->update(model);

	targets_ = new TargetPanel(this);
	targets_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	(void)connect(targets_, &TargetPanel::targetsChanged, this, &ArmSettings::targetSettingChanged);
	layout_->addWidget(targets_);
	targets_->update(model);

	for (int i = 0; i < model_.jointCount(); i++) {
		const JointDataModel& joint = model_.jointModel(i);
		addJoint(i, joint);
	}

	setLayout(layout_); 

	(void)connect(&model_, &ArmDataModel::dataChanged, this, &ArmSettings::modelDataChanged);
}

ArmSettings::~ArmSettings()
{
}

void ArmSettings::modelDataChanged()
{
	robot_->update(model_);
	targets_->update(model_);

	if (settings_.size() != model_.jointCount()) {
		clear();

		for (int i = 0; i < model_.jointCount(); i++) {
			addJoint(i, model_.jointModel(i));
		}
	}
	else {
		for (int i = 0; i < model_.jointCount(); i++) {
			settings_[i]->update(model_.jointModel(i));
		}
	}
}

void ArmSettings::targetSettingChanged()
{
	model_.clearTargets();
	for (const QPointF& pt : targets_->targets()) {
		model_.addTarget(pt);
	}
}

void ArmSettings::robotSettingChanged(ChangeType type)
{
	switch (type) {
	case ChangeType::ArmPos:
		model_.setArmPos(robot_->getArmPos());
		break;
	case ChangeType::BumperPos:
		model_.setBumperPos(robot_->getBumperPos());
		break;
	case ChangeType::BumperSize:
		model_.setBumperSize(robot_->getBumperSize());
		break;
	}
}

void ArmSettings::jointSettingChanged(int which, ChangeType type)
{
	JointDataModel dm = model_.jointModel(which);
	switch (type)
	{
	case ChangeType::ArmKeepout:
		{
			dm.clearKeepOut();
			for (QPair<double, double> p : settings_[which]->keepOuts()) {
				dm.addKeepOut(p);
			}
		}
		break;
	case ChangeType::ArmLength:
		dm.setLength(settings_[which]->length());
		break;

	case ChangeType::Current:
		dm.setAngle(settings_[which]->currentPos());
		break;

	case ChangeType::initialAngleition:
		dm.setinitialAngle(settings_[which]->initialAngle());
	}

	model_.replaceJointModel(which, dm);
}

void ArmSettings::clear()
{
	for (int i = 0; i < layout_->count() - 2; i++) {
		layout_->removeItem(layout_->itemAt(i));
	}

	for (int i = 0; i < settings_.size(); i++) {
		delete settings_[i];
	}
	settings_.clear();
}

void ArmSettings::addJoint(int n, const JointDataModel &model)
{
	QString name = "L" + QString::number(n + 1);
	OneArmSettings* l = new OneArmSettings(n, name, this);
	layout_->insertWidget(n, l);

	l->update(model);

	(void)connect(l, &OneArmSettings::settingsChanged, this, &ArmSettings::jointSettingChanged);
	l->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

	settings_.push_back(l);
}

