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
#pragma once

#include "ArmPath.h"
#include "NoEditDelegate.h"
#include "ArmDataModel.h"
#include <QtWidgets/QTreeWidget>

class WaypointWindow : public QTreeWidget
{
	Q_OBJECT

public:
	WaypointWindow(ArmDataModel &model_, QWidget* parent);

	bool isSelectedIndex(int index) const {
		return index == index_;
	}

	void setPath(std::shared_ptr<ArmPath> path) {
		if (path_ != path) {
			path_ = path;
			index_ = -1;
		}
	}

	std::shared_ptr<ArmPath> path() {
		return path_;
	}

	void setWaypoint(std::shared_ptr<ArmPath> path, int index) {
		assert(path == path_);
		index_ = index;
		modelDataChanged(ChangeType::PathPoint);
	}

	int getWaypoint() const {
		return index_;
	}

private:
	static constexpr const char* IndexTag = "Index";
	static constexpr const char* DistanceTag = "distance";

	static constexpr const char* XTag = "x";
	static constexpr const char* YTag = "y";
	static constexpr const char* HeadingTag = "heading";

private:
	QTreeWidgetItem* newItem(const QString& title, bool editable = true);
	void waypointParamChanged(QTreeWidgetItem* item, int column);

	void modelDataChanged(ChangeType type);

private:
	ArmDataModel& model_;
	std::shared_ptr<ArmPath> path_;
	int index_;
	bool ignore_;
};

