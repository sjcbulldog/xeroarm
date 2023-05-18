#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QSlider>
#include <QtCore/QSettings>
#include "PathsDisplayWidget.h"
#include "ArmSettings.h"
#include "ArmDisplay.h"

class ArmDataModel;

class CentralWidget : public QWidget
{
	Q_OBJECT
	
public:
	CentralWidget(ArmDataModel &model, QWidget* parent);
	~CentralWidget();

	void resetDisplay() {
		display_->resetDisplay();
	}

	ArmDisplay* display() {
		return display_;
	}

	QSplitter* getMainSplitter() { return main_;  }

	void pathSelected(std::shared_ptr<ArmPath> path);

signals:
	void changeTime(double t);
	void mouseMove(const Translation2d& pos);

private:
	void dataModelChanged();
	void timeChanged(int ms);
	void mouseMoved(const Translation2d& pos);

private:
	ArmDataModel& model_;
	ArmSettings* settings_;
	ArmDisplay* display_;
	QSplitter* main_;
	QSlider* slider_;
	std::shared_ptr<ArmPath> path_;
};

