#include "CentralWidget.h"
#include "ArmDataModel.h"
#include "PathsDisplayWidget.h"
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QJsonDocument>
#include <QtCore/QFile>
#include <QtWidgets/QBoxLayout>

CentralWidget::CentralWidget(ArmDataModel &model, QWidget *parent) : model_(model)
{
	QVBoxLayout* layout = new QVBoxLayout();
	main_ = new QSplitter(Qt::Vertical);
	layout->addWidget(main_);

	settings_ = new ArmSettings(model, this);
	settings_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Minimum);
	main_->addWidget(settings_);

	display_ = new ArmDisplay(model_, this);
	display_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	main_->addWidget(display_);

	slider_ = new QSlider(Qt::Horizontal);
	layout->addWidget(slider_);

	setLayout(layout);

	(void)connect(&model_, &ArmDataModel::dataChanged, this, &CentralWidget::dataModelChanged);
	(void)connect(slider_, &QSlider::valueChanged, this, &CentralWidget::timeChanged);
}

CentralWidget::~CentralWidget()
{
}

void CentralWidget::timeChanged(int ms)
{
	if (path_ == nullptr || path_->profile() == nullptr)
		return;

	double t = static_cast<double>(ms) / 100.0;
	Pose2dTrajectory pt = path_->profile()->getByTime(t);
	for (int i = 0; i < model_.jointCount(); i++) {
		model_.setJointAngle(i, pt.angles().at(i));
	}

	emit changeTime(t);
	display_->repaint();
}

void CentralWidget::pathSelected(std::shared_ptr<ArmPath> path)
{
	path_ = path;
	display_->setCurrentPath(path);

	if (path_->profile() != nullptr) {
		auto prof = path_->profile();
		slider_->setMaximum(static_cast<int>(prof->time() * 100));
	}
	slider_->setValue(0);
}

void CentralWidget::dataModelChanged()
{
	display_->resetDisplay();
}