#pragma once

#include "Translation2d.h"
#include <QtWidgets/QWidget>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include "ChangeType.h"

class ArmDataModel;

class RobotSettings : public QFrame
{
	Q_OBJECT

public:
	RobotSettings(QWidget* parent);

	void update(const ArmDataModel& model);

	Translation2d getArmPos();
	Translation2d getBumperPos();
	Translation2d getBumperSize();

signals:
	void settingsChanged(ChangeType type);

private:
	void armPosChanged();
	void bumperPosChanged();
	void bumperSizeChanged();

private:
	QLabel* title_label_;

	QLabel* bumper_pos_x_label_;
	QLineEdit* bumper_pos_x_;

	QLabel* bumper_pos_y_label_;
	QLineEdit* bumper_pos_y_;

	QLabel* bumper_size_x_label_;
	QLineEdit* bumper_size_x_;

	QLabel* bumper_size_y_label_;
	QLineEdit* bumper_size_y_;

	QLabel* arm_pos_x_label_;
	QLineEdit* arm_pos_x_;

	QLabel* arm_pos_y_label_;
	QLineEdit* arm_pos_y_;

};

