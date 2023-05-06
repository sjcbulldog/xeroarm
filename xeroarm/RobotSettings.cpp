#include "RobotSettings.h"
#include "ArmDataModel.h"
#include <QtWidgets/QGridLayout>
#include <QtGui/QDoubleValidator>

RobotSettings::RobotSettings(QWidget* parent) : QFrame(parent)
{
	QDoubleValidator* valid;
	QGridLayout* lay = new QGridLayout();

	setFrameShape(QFrame::StyledPanel);

	int row = 0;

	title_label_ = new QLabel("Robot/Target Settings");
	QFont font = title_label_->font();
	font.setBold(true);
	font.setPointSizeF(16.0);
	title_label_->setFont(font);
	lay->addWidget(title_label_, row, 0, 1, 2, Qt::AlignHCenter);
	row++;

	bumper_pos_x_label_ = new QLabel("Bumper Pos X");
	lay->addWidget(bumper_pos_x_label_, row, 0, Qt::AlignRight);

	bumper_pos_x_ = new QLineEdit();
	valid = new QDoubleValidator();
	bumper_pos_x_->setValidator(valid);
	bumper_pos_x_->setText("12.0");
	lay->addWidget(bumper_pos_x_, row, 1, Qt::AlignLeft);
	(void)connect(bumper_pos_x_, &QLineEdit::editingFinished, this, &RobotSettings::bumperPosChanged);
	row++;

	bumper_pos_y_label_ = new QLabel("Bumper Pos Y");
	lay->addWidget(bumper_pos_y_label_, row, 0, Qt::AlignRight);

	bumper_pos_y_ = new QLineEdit();
	valid = new QDoubleValidator();
	bumper_pos_y_->setValidator(valid);
	bumper_pos_y_->setText("12.0");
	lay->addWidget(bumper_pos_y_, row, 1, Qt::AlignLeft);
	(void)connect(bumper_pos_y_, &QLineEdit::editingFinished, this, &RobotSettings::bumperPosChanged);
	row++;

	bumper_size_x_label_ = new QLabel("Bumper Size X");
	lay->addWidget(bumper_size_x_label_, row, 0, Qt::AlignRight);

	bumper_size_x_ = new QLineEdit();
	valid = new QDoubleValidator();
	bumper_size_x_->setValidator(valid);
	bumper_size_x_->setText("12.0");
	lay->addWidget(bumper_size_x_, row, 1, Qt::AlignLeft);
	(void)connect(bumper_size_x_, &QLineEdit::editingFinished, this, &RobotSettings::bumperSizeChanged);
	row++;

	bumper_size_y_label_ = new QLabel("Bumper Size Y");
	lay->addWidget(bumper_size_y_label_, row, 0, Qt::AlignRight);

	bumper_size_y_ = new QLineEdit();
	valid = new QDoubleValidator();
	bumper_size_y_->setValidator(valid);
	bumper_size_y_->setText("12.0");
	lay->addWidget(bumper_size_y_, row, 1, Qt::AlignLeft);
	(void)connect(bumper_size_y_, &QLineEdit::editingFinished, this, &RobotSettings::bumperSizeChanged);
	row++;

	arm_pos_x_label_ = new QLabel("Arm Pos X");
	lay->addWidget(arm_pos_x_label_, row, 0, Qt::AlignRight);

	arm_pos_x_ = new QLineEdit();
	valid = new QDoubleValidator();
	arm_pos_x_->setValidator(valid);
	arm_pos_x_->setText("0.0");
	lay->addWidget(arm_pos_x_, row, 1, Qt::AlignLeft);
	(void)connect(arm_pos_x_, &QLineEdit::editingFinished, this, &RobotSettings::armPosChanged);
	row++;

	arm_pos_y_label_ = new QLabel("Arm Pos Y");
	lay->addWidget(arm_pos_y_label_, row, 0, Qt::AlignRight);

	arm_pos_y_ = new QLineEdit();
	valid = new QDoubleValidator();
	arm_pos_y_->setValidator(valid);
	arm_pos_y_->setText("0.0");
	lay->addWidget(arm_pos_y_, row, 1, Qt::AlignLeft);
	(void)connect(arm_pos_y_, &QLineEdit::editingFinished, this, &RobotSettings::armPosChanged);
	row++;

	setLayout(lay);
}

void RobotSettings::bumperSizeChanged()
{
	emit settingsChanged(ChangeType::BumperSize);
}

void RobotSettings::bumperPosChanged()
{
	emit settingsChanged(ChangeType::BumperPos);
}

void RobotSettings::armPosChanged()
{
	emit settingsChanged(ChangeType::ArmPos);
}

QPointF RobotSettings::getArmPos()
{
	return QPointF(arm_pos_x_->text().toDouble(), arm_pos_y_->text().toDouble());
}

QPointF RobotSettings::getBumperPos()
{
	return QPointF(bumper_pos_x_->text().toDouble(), bumper_pos_y_->text().toDouble());
}

QSizeF RobotSettings::getBumperSize()
{
	return QSizeF(bumper_size_x_->text().toDouble(), bumper_size_y_->text().toDouble());
}

void RobotSettings::update(const ArmDataModel& model)
{
	bumper_pos_x_->setText(QString::number(model.bumperPos().x(), 'f', 2));
	bumper_pos_y_->setText(QString::number(model.bumperPos().y(), 'f', 2));
	arm_pos_x_->setText(QString::number(model.armPos().x(), 'f', 2));
	arm_pos_y_->setText(QString::number(model.armPos().y(), 'f', 2));
}