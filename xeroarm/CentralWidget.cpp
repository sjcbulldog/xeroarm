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
	settings_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
	main_->addWidget(settings_);

	bottom_ = new QSplitter(Qt::Horizontal);
	main_->addWidget(bottom_);

	display_ = new ArmDisplay(model_, this);
	display_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	bottom_->addWidget(display_);

	paths_ = new PathsDisplayWidget(model);
	paths_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::MinimumExpanding);
	bottom_->addWidget(paths_);
	(void)connect(paths_, &PathsDisplayWidget::pathSelected, this, &CentralWidget::pathSelected);

	setLayout(layout);

	(void)connect(&model_, &ArmDataModel::dataChanged, this, &CentralWidget::dataModelChanged);
}

CentralWidget::~CentralWidget()
{
}

void CentralWidget::pathSelected(std::shared_ptr<ArmPath> path)
{
	display_->setCurrentPath(path);
}

void CentralWidget::dataModelChanged()
{
	display_->resetDisplay();
}