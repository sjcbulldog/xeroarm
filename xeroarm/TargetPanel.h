#pragma once

#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>

class ArmDataModel;

class TargetPanel : public QFrame
{
	Q_OBJECT

public:
	TargetPanel(QWidget* parent);
	void update(ArmDataModel& model);
	QVector<QPointF> targets();

signals:
	void targetsChanged();

private:
	static constexpr const char* AddHereText = "Add Target Here";

private:
	void targetDoubleClicked(QListWidgetItem* item);
	void targetChanged(QListWidgetItem* item);
	bool isTargetValid(const QString& text);

private:
	QLabel* target_label_;
	QListWidget* targets_;

	QString prev_text_;
	bool is_editing_target_;
};

