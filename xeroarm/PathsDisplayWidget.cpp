#include "PathsDisplayWidget.h"
#include "ArmDataModel.h"
#include <QtWidgets/QMenu>
#include <QtWidgets/QMessageBox>

PathsDisplayWidget::PathsDisplayWidget(ArmDataModel& model, QWidget* parent) : QTreeWidget(parent), model_(model)
{
	setHeaderHidden(true);
	setContextMenuPolicy(Qt::CustomContextMenu);
	connect(this, &QTreeWidget::customContextMenuRequested, this, &PathsDisplayWidget::prepareCustomMenu);
	connect(this, &QTreeWidget::currentItemChanged, this, &PathsDisplayWidget::selectedItemChanged);
	connect(this, &QTreeWidget::itemChanged, this, &PathsDisplayWidget::itemRenamed);

	connect(&model, &ArmDataModel::dataChanged, this, &PathsDisplayWidget::modelChanged);

	current_ = nullptr;
}

QTreeWidgetItem* PathsDisplayWidget::addPathToDisplay(std::shared_ptr<ArmPath> path)
{
	QTreeWidgetItem* item = new QTreeWidgetItem();
	item->setFlags(item->flags() | Qt::ItemIsEditable);
	item->setText(0, path->name());
	item->setData(0, Qt::UserRole, path->name());
	addTopLevelItem(item);

	return item;
}

void PathsDisplayWidget::modelChanged()
{
	clear();
	for (auto path : model_.getPaths()) {
		addPathToDisplay(path);
	}
}

void PathsDisplayWidget::prepareCustomMenu(const QPoint &pt)
{
	QMenu menu(this);
	QAction* act;

	current_ = itemAt(pt);
	if (current_ != nullptr) {
		act = new QAction(tr("Delete Path"));
		connect(act, &QAction::triggered, this, &PathsDisplayWidget::deletePath);
		menu.addAction(act);
	}

	act = new QAction(tr("Add Path"));
	connect(act, &QAction::triggered, this, &PathsDisplayWidget::addPath);
	menu.addAction(act);

	menu.exec(this->mapToGlobal(pt));
	current_ = nullptr;
}

QString PathsDisplayWidget::findName()
{
	QString name;
	int i = 1;

	while (true) {
		name = "Path_" + QString::number(i);
		if (!model_.hasPath(name))
			break;

		i++;
	}

	return name;
}

void PathsDisplayWidget::addPath()
{
	QString name = findName();

	auto path = std::make_shared<ArmPath>(name);
	QPointF inipt = model_.getInitialArmPos();
	path->addPoint(Pose2d(inipt.x(), inipt.y()));
	path->addPoint(Pose2d(40.0, 40.0, Rotation2d::fromDegrees(45.0)));
	model_.addPath(path);
}

void PathsDisplayWidget::deletePath()
{
	QString name = current_->text(0);
	model_.removePath(name);

	delete current_;
	current_ = nullptr;

	if (current_path_->name() == name) {
		current_path_ = nullptr;
		emit pathSelected(nullptr);
	}
}

void PathsDisplayWidget::selectedItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* prev)
{
	if (current == nullptr) {
		current_path_ = nullptr;
		emit pathSelected(nullptr);
	}
	else {
		auto path = model_.getPathByName(current->text(0));
		current_path_ = path;
		emit pathSelected(path);
	}
}

bool PathsDisplayWidget::isValidName(const QString& name)
{
	for (int i = 0; i < name.length(); i++) {
		if (!name.at(i).isLetterOrNumber() && name.at(i) != '_')
			return false;
	}

	return true;
}

void PathsDisplayWidget::itemRenamed(QTreeWidgetItem* item, int column)
{
	QString oldname = item->data(0, Qt::UserRole).toString();
	QString newname = item->text(0);

	if (oldname == newname)
		return;

	if (!isValidName(newname))
	{
		QMessageBox::critical(this, "Invalid Name", "The name '" + newname + "' is not a valid name.  A name must consist of letters, numbers, and underscore only.");
		item->setText(0, oldname);
		return;
	}

	if (model_.hasPath(newname)) 
	{
		QMessageBox::critical(this, "Invalid Name", "The name '" + newname + "' is not a valid name.  A path with that name already exists.");
		item->setText(0, oldname);
		return;
	}

	model_.renamePath(oldname, newname);
}