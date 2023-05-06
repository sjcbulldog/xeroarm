//
// Copyright 2022 Jack W. Griffin
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
// http ://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissionsand
// limitations under the License.
//
#include "WaypointWindow.h"
#include <QtWidgets/QMessageBox>

WaypointWindow::WaypointWindow(ArmDataModel &model, QWidget* parent) : QTreeWidget(parent), model_(model)
{
	index_ = -1;

	setHeaderHidden(true);
	setColumnCount(2);

	setItemDelegateForColumn(0, new NoEditDelegate(this));
	connect(this, &QTreeWidget::itemChanged, this, &WaypointWindow::waypointParamChanged);
	connect(&model_, &ArmDataModel::dataChanged, this, &WaypointWindow::modelDataChanged);

	ignore_ = false;
}

void WaypointWindow::waypointParamChanged(QTreeWidgetItem* item, int column)
{
	assert(column == 1);
	assert(item->text(0) != IndexTag);

	bool ok;
	double value = item->text(1).toDouble(&ok);

	if (!ok) 
	{
		QMessageBox::critical(this, "Invalid Number", "The string '" + item->text(1) + "' is not a valid number");
		modelDataChanged(ChangeType::PathPoint);
	}
	else
	{
		const Pose2d pt = path_->at(index_);
		Pose2d newpt;

		if (item->text(0) == XTag)
		{
			newpt = Pose2d(Translation2d(value, pt.getTranslation().getY()), pt.getRotation());
		}
		else if (item->text(0) == YTag)
		{
			newpt = Pose2d(Translation2d(pt.getTranslation().getX(), value), pt.getRotation());
		}
		else if (item->text(0) == HeadingTag) 
		{
			newpt = Pose2d(Translation2d(pt.getTranslation().getX(), pt.getTranslation().getY()), Rotation2d::fromDegrees(value));
		}
		path_->replacePoint(index_, newpt);

		ignore_ = true;
		model_.pathPointChanged();
		ignore_ = false;
	}
}

QTreeWidgetItem* WaypointWindow::newItem(const QString& title, bool editable)
{
	QTreeWidgetItem* item = new QTreeWidgetItem();
	item->setText(0, title);
	if (editable) {
		item->setFlags(item->flags() | Qt::ItemIsEditable);
	}

	return item;
}

void WaypointWindow::modelDataChanged(ChangeType type)
{
	clear();

	if (path_ != nullptr && index_ >= 0 && index_ < path_->size() && type == ChangeType::PathPoint && !ignore_)
	{
		QTreeWidgetItem* item;
		const auto& pt = path_->at(index_);

		item = newItem(IndexTag, false);
		item->setText(1, QString::number(index_));
		addTopLevelItem(item);

		item = newItem(XTag);
		item->setText(1, QString::number(pt.getTranslation().getX(), 'f', 2));
		addTopLevelItem(item);

		item = newItem(YTag);
		item->setText(1, QString::number(pt.getTranslation().getY(), 'f', 2));
		addTopLevelItem(item);

		item = newItem(HeadingTag);
		item->setText(1, QString::number(pt.getRotation().toDegrees(), 'f', 2));
		addTopLevelItem(item);
	}
}
