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

	maxv_label_ = new QLabel("Max Acceleartion");
	lay->addWidget(maxv_label_, row, 0, Qt::AlignRight);

	maxa_ = new QLineEdit();
	valid = new QDoubleValidator();
	maxa_->setValidator(valid);
	maxa_->setText("0.0");
	lay->addWidget(maxa_, row, 1, Qt::AlignLeft);
	(void)connect(maxa_, &QLineEdit::editingFinished, this, &OneArmSettings::maxaChanged);
	row++;

	setLayout(lay);
}

OneArmSettings::~OneArmSettings()
{
}

void OneArmSettings::update(const JointDataModel& model)
{
	length_->setText(QString::number(model.length(), 'f', 2));
	current_->setText(QString::number(model.angle(), 'f', 2));
	initial_pos_->setText(QString::number(model.initialAngle(), 'f', 2));
	maxv_->setText(QString::number(model.maxVelocity(), 'f', 2));
	maxa_->setText(QString::number(model.maxAccel(), 'f', 2));
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
