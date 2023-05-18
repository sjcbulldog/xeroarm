#pragma once

#include <QtWidgets/QFrame>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include "ChangeType.h"
#include "JointDataModel.h"

class OneArmSettings : public QFrame
{
	Q_OBJECT

public:
	OneArmSettings(int which, const QString &title, QWidget* parent);
	~OneArmSettings();

	double length() {
		return length_->text().toDouble();
	}

	double initialAngle() {
		return initial_pos_->text().toDouble();
	}

	double currentPos() {
		return current_->text().toDouble();
	}

	double maxVelocity() {
		return maxv_->text().toDouble();
	}

	double maxAccel() {
		return maxa_->text().toDouble();
	}

	void update(const JointDataModel& model);

signals:
	void settingsChanged(int which, ChangeType type);

private:
	void lengthChanged();
	void initialPosChanged();
	void currentChanged();
	void maxvChanged();
	void maxaChanged();

private:
	static constexpr const char* AddHereText = "Add New Keepout";

private:
	int which_;

	QString title_;
	QLabel* title_label_;

	QLabel* length_label_;
	QLineEdit* length_;

	QLabel* initial_pos_label_;
	QLineEdit* initial_pos_;

	QLabel* current_label_;
	QLineEdit* current_;

	QLabel* maxv_label_;
	QLineEdit* maxv_;

	QLabel* maxa_label_;
	QLineEdit* maxa_;

	QString prev_text_;
	bool is_editing_keepout_;
};

