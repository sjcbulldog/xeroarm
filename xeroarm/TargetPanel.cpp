#include "TargetPanel.h"
#include "ArmDataModel.h"
#include <QtWidgets/QBoxLayout>
#include <QtWidgets/QMessageBox>

TargetPanel::TargetPanel(QWidget* parent) : QFrame(parent)
{
	setFrameShape(QFrame::StyledPanel);

	QVBoxLayout* layout = new QVBoxLayout();
	layout->setAlignment(Qt::AlignTop);

	target_label_ = new QLabel("Targets");
	QFont font = target_label_->font();
	font.setBold(true);
	font.setPointSizeF(16.0);
	target_label_->setFont(font);
	target_label_->setMaximumWidth(160);
	layout->addWidget(target_label_);


	is_editing_target_ = false;

	targets_ = new QListWidget();
	targets_->setMaximumHeight(400);
	targets_->setMaximumWidth(240);
	targets_->setEditTriggers(QAbstractItemView::EditTrigger::DoubleClicked);
	layout->addWidget(targets_);

	(void)connect(targets_, &QListWidget::itemDoubleClicked, this, &TargetPanel::targetDoubleClicked);
	(void)connect(targets_, &QListWidget::itemChanged, this, &TargetPanel::targetChanged);

	setLayout(layout);
}

bool TargetPanel::isTargetValid(const QString& text)
{
	QStringList items = text.split(',');
	if (items.length() != 2)
		return false;

	QDoubleValidator val;
	int pos = 0;
	QString str = items.at(0);
	if (val.validate(str, pos) != QValidator::Acceptable)
		return false;

	pos = 0;
	str = items.at(1);
	if (val.validate(str, pos) != QValidator::Acceptable)
		return false;

	return true;
}

void TargetPanel::targetDoubleClicked(QListWidgetItem* item)
{
	is_editing_target_ = true;
	prev_text_ = item->text();
	qDebug() << "TargetPanel::targetDoubleClicked(), prev_text_ = '" << prev_text_ << "'";
}

void TargetPanel::targetChanged(QListWidgetItem* item)
{
	qDebug() << "TargetPanel::targetChanged(), prev_text_ = '" << prev_text_ << "', new text " << item->text();
	if (isTargetValid(item->text())) {
		if (prev_text_ == AddHereText) {
			QListWidgetItem* item = new QListWidgetItem(AddHereText);
			item->setFlags(Qt::ItemFlag::ItemIsEditable | item->flags());
			targets_->addItem(item);
		}
		emit targetsChanged();
		is_editing_target_ = false;
	}
	else {
		QMessageBox::critical(targets_, "Error In Target Entry", "The target entry entered is not valid.  It should be two floating point numbers seperated by a common");
		targets_->openPersistentEditor(item);
	}
}

void TargetPanel::update(ArmDataModel& model)
{

	if (!is_editing_target_) {
		targets_->clear();

		for (const Translation2d& pt : model.targets()) {
			QString entry = QString::number(pt.getX()) + ", " + QString::number(pt.getY());
			QListWidgetItem* item = new QListWidgetItem(entry);
			item->setFlags(item->flags() | Qt::ItemFlag::ItemIsEditable);
			targets_->addItem(item);
		}

		QListWidgetItem* item = new QListWidgetItem(AddHereText);
		item->setFlags(item->flags() | Qt::ItemFlag::ItemIsEditable);
		targets_->addItem(item);
	}
}

QVector<Translation2d> TargetPanel::targets()
{
	QVector<Translation2d> ret;
	QVector<int> remove;

	for (int i = 0; i < targets_->count(); i++) {
		QString txt = targets_->item(i)->text();
		if (txt == AddHereText)
			break;

		QStringList items = txt.split(',');

		if (items.count() != 2) {
			remove.push_back(i);
		}
		else {
			double x = items.at(0).toDouble();
			double y = items.at(1).toDouble();
			ret.push_back(Translation2d(x, y));
		}
	}

	//
	// Remove any items that are mal formed
	//
	while (remove.count() > 0) {
		int which = remove.at(remove.count() - 1);
		delete targets_->takeItem(which);
		remove.pop_back();
	}

	return ret;
}