#include "xeroarm.h"
#include <QtCore/QCoreApplication>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStatusBar>
#include <QtGui/QCloseEvent>

xeroarm::xeroarm(QWidget *parent) : QMainWindow(parent)
{
	createWindows();
	createMenus();

	if (settings_.contains(GeometrySetting))
		restoreGeometry(settings_.value(GeometrySetting).toByteArray());

	if (settings_.contains(WindowStateSetting))
		restoreState(settings_.value(WindowStateSetting).toByteArray());

	if (settings_.contains(MainSplitterSettings)) {
		QList<int> sizes ;
		QList<QVariant> stored = settings_.value(MainSplitterSettings).toList();
		for (const QVariant& v : stored) {
			sizes.push_back(v.toInt());
		}

		central_->getMainSplitter()->setSizes(sizes);
	}

	if (settings_.contains(PlotSplitterSettings)) {
		QList<int> sizes;
		QList<QVariant> stored = settings_.value(PlotSplitterSettings).toList();
		for (const QVariant& v : stored) {
			sizes.push_back(v.toInt());
		}

		plot_win_->setSizes(sizes);
	}

	if (QCoreApplication::arguments().count() > 1) {
		QString filename = QCoreApplication::arguments().at(1);
		QString error;

		if (!model_.load(filename, error)) {
			QMessageBox::warning(this, "Error", "Error loading file - " + error);
		}
		else {
			filename_ = filename;
			central_->resetDisplay();
		}
	}

	status_text_ = new QLabel("Idle");
	statusBar()->addWidget(status_text_);

	(void)connect(&model_, &ArmDataModel::progress, this, &xeroarm::progress);
}

xeroarm::~xeroarm()
{
}

void xeroarm::progress(const QString& msg)
{
	status_text_->setText(msg);
}

void xeroarm::createWindows()
{
	central_ = new CentralWidget(model_, this);
	setCentralWidget(central_);

	path_display_dock_ = new QDockWidget(tr("Paths"));
	path_display_ = new PathsDisplayWidget(model_);
	path_display_dock_->setObjectName("paths");
	path_display_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	path_display_dock_->setWidget(path_display_);
	addDockWidget(Qt::RightDockWidgetArea, path_display_dock_);

	waypoint_display_dock_ = new QDockWidget(tr("Waypoint"));
	waypoint_display_ = new WaypointWindow(model_, nullptr);
	waypoint_display_dock_->setObjectName("paths");
	waypoint_display_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
	waypoint_display_dock_->setWidget(waypoint_display_);
	addDockWidget(Qt::RightDockWidgetArea, waypoint_display_dock_);

	plot_win_ = new PlotWindow(nullptr);
	dock_plot_win_ = new QDockWidget(tr("Plot"));
	dock_plot_win_->setObjectName("plot");
	dock_plot_win_->setAllowedAreas(Qt::TopDockWidgetArea | Qt::BottomDockWidgetArea);
	dock_plot_win_->setWidget(plot_win_);
	addDockWidget(Qt::BottomDockWidgetArea, dock_plot_win_);
	dock_plot_win_->hide();

	(void)connect(path_display_, &PathsDisplayWidget::pathSelected, central_, &CentralWidget::pathSelected);
	(void)connect(path_display_, &PathsDisplayWidget::pathSelected, waypoint_display_, &WaypointWindow::setPath);
	(void)connect(path_display_, &PathsDisplayWidget::pathSelected, plot_win_, &PlotWindow::setPath);

	(void)connect(central_->display(), &ArmDisplay::pathPointSelected, waypoint_display_, &WaypointWindow::setWaypoint);
}

void xeroarm::closeEvent(QCloseEvent* ev)
{
	if (model_.isDirty()) {
		QMessageBox::StandardButton ans = QMessageBox::question(this, "Save Changes", "There are unsaved changes, do you really want to quit?");
		if (ans == QMessageBox::StandardButton::No) {
			ev->ignore();
			return;
		}
	}

	settings_.setValue(GeometrySetting, saveGeometry());
	settings_.setValue(WindowStateSetting, saveState());

	QList<QVariant> params;
	for (auto size : central_->getMainSplitter()->sizes()) {
		params.push_back(size);
	}
	settings_.setValue(MainSplitterSettings, params);

	params.clear();
	for (auto size : plot_win_->sizes()) {
		params.push_back(size);
	}
	settings_.setValue(PlotSplitterSettings, params);
}

void xeroarm::createMenus()
{
	QAction* act;

	file_menu_ = new QMenu(tr("&File"));
	menuBar()->addMenu(file_menu_);

	act = file_menu_->addAction("Open ...");
	connect(act, &QAction::triggered, this, &xeroarm::openFile);

	file_menu_->addSeparator();

	act = file_menu_->addAction("Close ...");
	connect(act, &QAction::triggered, this, &xeroarm::closeFile);

	file_menu_->addSeparator();

	act = file_menu_->addAction("Save");
	connect(act, &QAction::triggered, this, &xeroarm::saveFile);

	act = file_menu_->addAction("Save As ...");
	connect(act, &QAction::triggered, this, &xeroarm::saveAsFile);

	window_menu_ = new QMenu(tr("&Windows"));
	menuBar()->addMenu(window_menu_);
	window_menu_->addAction(path_display_dock_->toggleViewAction());
	window_menu_->addAction(waypoint_display_dock_->toggleViewAction());
	window_menu_->addAction(dock_plot_win_->toggleViewAction());
	window_menu_->addSeparator();
}

void xeroarm::saveFile()
{
	if (filename_.isEmpty()) {
		saveAsFile();
	}
	else {
		save(filename_);
	}
}

void xeroarm::saveAsFile()
{
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Path File"), "", tr("Arm File (*.xeroarm);;All Files (*)"));
	if (filename.length() == 0) {
		QMessageBox::warning(this, "No File Selected", "No output filename was selected, file not saved");
	}
	else {
		save(filename);
		filename_ = filename;
	}
}

void xeroarm::openFile()
{
	QString filename = QFileDialog::getOpenFileName(this, tr("Save Path File"), "", tr("Arm File (*.xeroarm);;All Files (*)"));
	if (filename.length() > 0) {
		QString error;

		if (!model_.load(filename, error)) {
			QMessageBox::warning(this, "Error", "Error loading file - " + error);
		}
		else {
			filename_ = filename;
			central_->resetDisplay();
		}
	}
}

void xeroarm::closeFile()
{
	model_.clear();
}

void xeroarm::save(const QString& path)
{
	filename_ = path;
	QString error;
	if (!model_.save(path, error)) {
		QMessageBox::warning(this, "Error", "Error saving file - " + error);
	}
}

void xeroarm::resetView()
{
	central_->resetDisplay();
}
