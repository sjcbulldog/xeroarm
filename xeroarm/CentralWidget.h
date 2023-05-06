#pragma once

#include <QtWidgets/QWidget>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QSplitter>
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
	QSplitter* getBottomSplitter() { return bottom_; }

	void pathSelected(std::shared_ptr<ArmPath> path);

private:
	void dataModelChanged();

private:
	ArmDataModel& model_;
	ArmSettings* settings_;
	ArmDisplay* display_;
	QSplitter* main_;
	QSplitter* bottom_;
};

