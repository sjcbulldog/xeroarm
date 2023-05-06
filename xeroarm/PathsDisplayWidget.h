#pragma once

#include <QtWidgets/QTreeWidget>

class ArmDataModel;
class ArmPath;

class PathsDisplayWidget : public QTreeWidget
{
	Q_OBJECT

public:
	PathsDisplayWidget(ArmDataModel &model, QWidget *parent = nullptr);

signals:
	void pathSelected(std::shared_ptr<ArmPath> path);

private:
	void selectedItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* prev);
	void prepareCustomMenu(const QPoint &pt);

	void deletePath();
	void addPath();

	QString findName();
	void itemRenamed(QTreeWidgetItem* item, int column);
	bool isValidName(const QString& name);

	void modelChanged();

	QTreeWidgetItem *addPathToDisplay(std::shared_ptr<ArmPath> path);

private:
	ArmDataModel& model_;
	QTreeWidgetItem* current_;
	std::shared_ptr<ArmPath> current_path_;
};

