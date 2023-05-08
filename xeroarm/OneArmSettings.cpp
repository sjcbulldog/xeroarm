#include "OneArmSettings.h"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QMessageBox>
#include <QtGui/QDoubleValidator>
#include <QtGui/QMouseEvent>

OneArmSettings::OneArmSettings(int which, const QString &title, QWidget *parent) : QFrame(parent)
{
	QGridLayout* lay = new QGridLayout();
	lay->setAlignment(Qt::AlignTop);

	int row = 0;

	setFrameShape(QFrame::StyledPanel);

	which_ = which;
	title_ = title;

	title_label_ = new QLabel(title);
	QFont font = title_label_->font();
	font.setBold(true);
	font.setPointSizeF(16.0);
	title_label_->setFont(font);
	lay->addWidget(title_label_, row, 0, 1, 2, Qt::AlignHCenter);
	row++;

	length_label_ = new QLabel("Length");
	lay->addWidget(length_label_, row, 0, Qt::AlignRight);

	length_ = new QLineEdit();
	QDoubleValidator* valid = new QDoubleValidator();
	length_->setValidator(valid);
	length_->setText("30.0");
	lay->addWidget(length_, row, 1, Qt::AlignLeft);
	(void)connect(length_, &QLineEdit::editingFinished, this, &OneArmSettings::lengthChanged);
	row++;

	initial_pos_label_ = new QLabel("Initial Position");
	lay->addWidget(initial_pos_label_, row, 0, Qt::AlignRight);

	initial_pos_ = new QLineEdit();
	valid = new QDoubleValidator();
	initial_pos_->setValidator(valid);
	initial_pos_->setText("0.0");
	lay->addWidget(initial_pos_, row, 1, Qt::AlignLeft);
	(void)connect(initial_pos_, &QLineEdit::editingFinished, this, &OneArmSettings::initialPosChanged);
	row++;

	current_label_ = new QLabel("Current Position");
	lay->addWidget(current_label_, row, 0, Qt::AlignRight);

	current_ = new QLineEdit();
	valid = new QDoubleValidator();
	current_->setValidator(valid);
	current_->setText("0.0");
	lay->addWidget(current_, row, 1, Qt::AlignLeft);
	(void)connect(current_, &QLineEdit::editingFinished, this, &OneArmSettings::currentChanged);
	row++;

	maxv_label_ = new QLabel("Max Velocity");
	lay->addWidget(maxv_label_, row, 0, Qt::AlignRight);

	maxv_ = new QLineEdit();
	valid = new QDoubleValidator();
	maxv_->setValidator(valid);
	maxv_->setText("0.0");
	lay->addWidget(maxv_, row, 1, Qt::AlignLeft);
	(void)connect(maxv_, &QLineEdit::editingFinished, this, &OneArmSettings::maxvChanged);
	row++;

	maxa_ = new QLineEdit();
	valid = new QDoubleValidator();
	maxa_->setValidator(valid);
	maxa_->setText("0.0");
	lay->addWidget(maxa_, row, 1, Qt::AlignLeft);
	(void)connect(maxa_, &QLineEdit::editingFinished, this, &OneArmSettings::maxaChanged);
	row++;

	keepout_label_ = new QLabel("Keep Out Regions");
	lay->addWidget(keepout_label_, row, 0, Qt::AlignRight);

	keepout_ = new QListWidget();
	keepout_->setMaximumHeight(100);
	keepout_->setMaximumWidth(160);
	keepout_->setEditTriggers(QAbstractItemView::EditTrigger::DoubleClicked);
	lay->addWidget(keepout_, row, 1, Qt::AlignLeft);
	(void)connect(keepout_, &QListWidget::itemDoubleClicked, this, &OneArmSettings::keepOutDoubleClicked);
	(void)connect(keepout_, &QListWidget::itemChanged, this, &OneArmSettings::keepOutChanged);
	row++;

	setLayout(lay);
}

OneArmSettings::~OneArmSettings()
{
}

void OneArmSettings::keepOutDoubleClicked(QListWidgetItem *item)
{
	is_editing_keepout_ = true;
	prev_text_ = item->text();
}

void OneArmSettings::keepOutChanged(QListWidgetItem* item)
{
	if (isKeepOutValid(item->text())) {
		if (prev_text_ == AddHereText) {
			QListWidgetItem* item = new QListWidgetItem(AddHereText);
			item->setFlags(Qt::ItemFlag::ItemIsEditable | item->flags());
			keepout_->addItem(item);
		}
		emit settingsChanged(which_, ChangeType::ArmKeepout);
	}
	else {
		QMessageBox::critical(keepout_, "Error In Keepout Entry", "The keepout entry entered is not valid.  It should be two floating point numbers seperated by a common");
		keepout_->openPersistentEditor(item);
	}
}

bool OneArmSettings::isKeepOutValid(const QString& text)
{
	QStringList items = text.split(',');
	if (items.length() != 2)
		return false;

	QDoubleValidator val;
	int pos = 0;
	QString str = items.at(0).trimmed();
	if (val.validate(str, pos) != QValidator::Acceptable)
		return false;

	pos = 0;
	str = items.at(1).trimmed();
	if (val.validate(str, pos) != QValidator::Acceptable)
		return false;

	return true;
}

void OneArmSettings::update(const JointDataModel& model)
{
	length_->setText(QString::number(model.length(), 'f', 2));
	current_->setText(QString::number(model.angle(), 'f', 2));
	initial_pos_->setText(QString::number(model.initialAngle(), 'f', 2));
	maxv_->setText(QString::number(model.maxVelocity(), 'f', 2));
	maxa_->setText(QString::number(model.maxAccel(), 'f', 2));

	keepout_->clear();
	for (int i = 0; i < model.keepOutCount(); i++) {
		QPair<double, double> keep = model.keepOut(i);
		QString entry = QString::number(keep.first, 'f', 2) + ", " + QString::number(keep.second, 'f', 2);
		QListWidgetItem* item = new QListWidgetItem(entry);
		item->setFlags(Qt::ItemFlag::ItemIsEditable | item->flags());
		keepout_->addItem(item);
	}

	QListWidgetItem* item = new QListWidgetItem(AddHereText);
	item->setFlags(Qt::ItemFlag::ItemIsEditable | item->flags());
	keepout_->addItem(item);
}

void OneArmSettings::lengthChanged()
{
	emit settingsChanged(which_, ChangeType::ArmLength);
}

void OneArmSettings::initialPosChanged()
{
	emit settingsChanged(which_, ChangeType::InitialAngle);
}

void OneArmSettings::currentChanged()
{
	emit settingsChanged(which_, ChangeType::CurrentAngle);
}

void OneArmSettings::maxvChanged()
{
	emit settingsChanged(which_, ChangeType::MaxVelocity);
}

void OneArmSettings::maxaChanged()
{
	emit settingsChanged(which_, ChangeType::MaxAccel);
}

QVector<QPair<double, double>> OneArmSettings::keepOuts()
{
	QVector<QPair<double, double>> ret;
	QVector<int> remove;

	for (int i = 0; i < keepout_->count(); i++) {
		QString txt = keepout_->item(i)->text();
		QStringList items = txt.split(',');

		if (items.count() != 2) {
			remove.push_back(i);
		}
		else {
			double minv = items.at(0).toDouble();
			double maxv = items.at(1).toDouble();
			ret.push_back(std::make_pair(minv, maxv));
		}
	}

	//
	// Remove any items that are mal formed
	//
	while (remove.count() > 0) {
		int which = remove.at(remove.count() - 1);
		delete keepout_->takeItem(which);
		remove.pop_back();
	}

	return ret;
}
