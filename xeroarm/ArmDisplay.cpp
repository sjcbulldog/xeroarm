#include "ArmDisplay.h"
#include "ArmDataModel.h"
#include <QtWidgets/QMessageBox>
#include <QtGui/QPainter>
#include <QtGui/QMouseEvent>

QVector<QColor> ArmDisplay::colors_ =
{
	QColor(0, 0, 255),
	QColor(0, 255, 0),
	QColor(255, 0, 0),
	QColor(255, 255, 0),
	QColor(0, 255, 255),
	QColor(255, 0, 255)
};

ArmDisplay::ArmDisplay(ArmDataModel &model, QWidget* parent) : QWidget(parent), model_(model)
{
	setMinimumSize(800, 600);

	margins_ = QMargins(10, 10, 10, 40);

	calcTransforms();

	angles_loc_ = QPoint(10, 10);

	setFocusPolicy(Qt::ClickFocus);

	triangle_ =
	{
		{ TriangleSize, 0.0},
		{ -TriangleSize / 2.0, TriangleSize / 2.0},
		{ -TriangleSize / 2.0, -TriangleSize / 2.0 }
	};

	(void)connect(&model_, &ArmDataModel::dataChanged, this, &ArmDisplay::redraw);
}

ArmDisplay::~ArmDisplay()
{
}

void ArmDisplay::keyPressEvent(QKeyEvent* ev)
{
	if (ev->key() == Qt::Key::Key_R)
	{
		model_.setToInitialArmPos();
		repaint();
	}
	else if (ev->key() == Qt::Key::Key_J) {
		JointDataModel model(30.0, 0.0);
		model_.addJointModel(model);
	}
	else if (ev->key() == Qt::Key::Key_G) {
		model_.generateTrajectories();
	}
	else if (ev->key() == Qt::Key::Key_Insert)
	{
		if (path_ != nullptr && selected_ != -1) {
			const Pose2d& p1 = path_->at(selected_);
			const Pose2d& p2 = path_->at(selected_ + 1);

			Rotation2d r = Rotation2d::fromRadians((p1.getRotation().toRadians() + p2.getRotation().toRadians()) / 2.0);
			Translation2d t((p1.getTranslation().getX() + p2.getTranslation().getX()) / 2.0, (p1.getTranslation().getY() + p2.getTranslation().getY()) / 2.0);
			Pose2d newpt(t, r);

			path_->insertPoint(selected_, newpt);

			// Move selected waypoint to the one we just created
			selected_++;
			emit pathPointSelected(path_, selected_);
			repaint();
		}
	}
}

bool ArmDisplay::hitTest(const QPointF &pt, int &index, bool& center)
{
	for (int i = 0; i < path_->count(); i++) {
		Pose2d p = path_->at(i);
		if (MathUtils::epsilonEqual(p.getTranslation().getX(), pt.x(), 1.5) && MathUtils::epsilonEqual(p.getTranslation().getY(), pt.y(), 1.5)) {
			index = i;
			center = true;
			return true;
		}
	}

	for (int i = 0; i < path_->count(); i++) {
		Pose2d p = path_->at(i);
		if (MathUtils::epsilonEqual(p.getTranslation().getX(), pt.x(), 3.0) && MathUtils::epsilonEqual(p.getTranslation().getY(), pt.y(), 3.0)) {
			index = i;
			center = false;
			return true;
		}
	}

	return false;
}

void ArmDisplay::mousePressEvent(QMouseEvent* ev)
{
	selected_ = -1;

	QTransform inv = xform_.inverted();
	QPointF pt = inv.map(ev->pos());

	if (ev->buttons() == Qt::RightButton) {
		auto angles = model_.arm().inverseKinematicsJacobian(Translation2d(pt.x(), pt.y()));
		if (angles.isEmpty()) {
			QMessageBox::critical(this, "Error", "Cannot find a solution for the point selected");
		}
		else {
			for (int i = 0; i < angles.count(); i++) {
				model_.setJointAngle(i, angles[i]);
			}
		}
	}
	else if (ev->buttons() == Qt::LeftButton && path_ != nullptr && xform_.isInvertible()) {
		QTransform inv = xform_.inverted();
		QPointF pt = inv.map(ev->pos());

		int index;
		bool center;

		if (hitTest(pt, index, center)) {
			selected_ = index;
			emit pathPointSelected(path_, index);
			rotating_ = !center;
			dragging_ = center;
		}
		else {
			rotating_ = false;
			dragging_ = false;
		}
	}

	repaint();
}

void ArmDisplay::mouseReleaseEvent(QMouseEvent* ev)
{
	rotating_ = false;
	dragging_ = false;
}

void ArmDisplay::mouseMoveEvent(QMouseEvent* ev)
{
	if (path_ != nullptr) {
		QTransform inv = xform_.inverted();
		QPointF mpt = inv.map(ev->pos());

		if (dragging_)
		{
			Pose2d old = path_->at(selected_);
			Translation2d t(mpt.x(), mpt.y());
			Pose2d npt(t, old.getRotation());
			path_->replacePoint(selected_, npt);
			model_.pathPointChanged();
		}
		else if (rotating_)
		{
			const Pose2d& pt = path_->at(selected_);
			double angle = std::atan2(mpt.y() - pt.getTranslation().getY(), mpt.x() - pt.getTranslation().getX()) - MathUtils::kPI / 2;
			Rotation2d r = Rotation2d::fromRadians(angle);
			Pose2d npt(pt.getTranslation(), r);
			path_->replacePoint(selected_, npt);
			model_.pathPointChanged();
		}
	}
}

void ArmDisplay::resizeEvent(QResizeEvent* ev)
{
	calcTransforms();
}

void ArmDisplay::resetDisplay()
{
	calcTransforms();
	repaint();
}

QRectF ArmDisplay::armBounds()
{
	double length = model_.arm().maxArmLength();

	double armxmin = model_.armPos().getX() - length;
	double armxmax = model_.armPos().getX() + length;
	double armymin = (model_.armPos().getY() < 0.0) ? model_.armPos().getY() : 0.0;
	double armymax = model_.armPos().getY() + length;

	return QRectF(armxmin, armymin, armxmax - armxmin, armymax - armymin);
}

QRectF ArmDisplay::bumperBounds()
{
	return QRectF(model_.bumperPos().getX(), model_.bumperPos().getY(), model_.bumperSize().getX(), model_.bumperSize().getY());
}

QRectF ArmDisplay::targetBounds()
{
	QRectF r;

	if (model_.targets().count() == 0)
		return r;

	double xmin = model_.targets().at(0).getX();
	double xmax = model_.targets().at(0).getX();
	double ymin = model_.targets().at(0).getY();
	double ymax = model_.targets().at(0).getY();

	for (int i = 1; i < model_.targets().count(); i++) {
		Translation2d t = model_.targets().at(i);
		xmin = std::min(xmin, t.getX());
		xmax = std::max(xmax, t.getX());
		ymin = std::min(ymin, t.getY());
		ymax = std::min(ymax, t.getY());
	}
	
	return QRect(xmin, ymin, xmax - xmin, ymax - ymin);
}

void ArmDisplay::calcTransforms()
{
	QRectF bounds = armBounds();
	bounds |= targetBounds();
	bounds |= bumperBounds();

	double scalex = (width() - margins_.left() - margins_.right()) / bounds.width() ;
	double scaley = (height() - margins_.top() - margins_.bottom()) / bounds.height() ;

	scale_ = std::min(scalex, scaley);

	int x = width() * -bounds.x() / bounds.width();
	int y = height() - margins_.bottom();
	origin_ = QPoint(x, y);

	xform_ = QTransform();
	xform_.translate(origin_.x(), origin_.y());
	xform_.scale(scale_, -scale_);
}

void ArmDisplay::paintEvent(QPaintEvent* ev)
{
	QPainter p(this);

	QBrush br(QColor(192, 192, 192));
	p.setBrush(br);
	p.drawRect(0, 0, width(), height());

	p.save();

	p.setTransform(xform_);

	drawOrigin(p);
	drawBumpers(p);
	drawTargets(p);
	drawArms(p);
	drawCurrentPath(p);

	p.restore();
}

void ArmDisplay::drawOnePoint(QPainter& paint, const Pose2d& pt, bool selected)
{
	paint.save();

	if (selected)
	{
		QPen pen(QColor(0xff, 0xff, 0x00, 0xff));
		pen.setWidthF(0.2);
		paint.setPen(pen);
	}
	else
	{
		QPen pen(QColor(0x00, 0x00, 0x00, 0xFF));
		pen.setWidthF(0.2);
		paint.setPen(pen);
	}

	QBrush brush(QColor(0xFF, 0xA5, 0x00, 0xFF));
	paint.setBrush(brush);

	QTransform xform = paint.transform();
	xform.translate(pt.getTranslation().getX(), pt.getTranslation().getY());
	xform.rotate(pt.getRotation().toDegrees());
	paint.setTransform(xform);
	paint.drawPolygon(&triangle_[0], 3);

	paint.restore();
}

QVector<std::shared_ptr<SplinePair>>  ArmDisplay::computeSplinesForPath(std::shared_ptr<ArmPath> path)
{
	QVector<std::shared_ptr<SplinePair>> splines;

	for (int i = 0; i < path->size() - 1; i++) {
		const Pose2d& p1 = path->at(i);
		const Pose2d& p2 = path->at(i + 1);
		auto pair = std::make_shared<SplinePair>(p1, p2);
		splines.push_back(pair);
	}

	return splines;
}

void ArmDisplay::drawSplines(QPainter& p)
{
	p.save();
	auto splines = computeSplinesForPath(path_);
	for (int i = 0; i < splines.size(); i++)
		drawSpline(p, splines[i]);
	p.restore();
}

void ArmDisplay::findSplineStep(std::shared_ptr<SplinePair> pair)
{
	pair->setStep(0.01);
}

void ArmDisplay::drawSpline(QPainter& paint, std::shared_ptr<SplinePair> pair)
{
	double px, py;
	QColor c(0xF0, 0x80, 0x80, 0xFF);

	QBrush brush(c);
	paint.setBrush(brush);

	QPen pen(c);
	pen.setWidthF(0.2);
	paint.setPen(pen);

	if (!pair->hasStep())
		findSplineStep(pair);

	for (float t = 0.0f; t < 1.0f; t += pair->step())
	{
		Translation2d loc = pair->evalPosition(t);
		Rotation2d heading = pair->evalHeading(t);

		QPointF qp(loc.getX(), loc.getY());
		paint.drawPoint(qp);
	}
}

void ArmDisplay::drawPoints(QPainter& p)
{
	for (int i = 0; i < path_->count(); i++) {
		drawOnePoint(p, path_->at(i), i == selected_);
	}
}

void ArmDisplay::drawCurrentPath(QPainter& p)
{
	if (path_ == nullptr)
		return;
	
	drawSplines(p);
	drawPoints(p);
}

void ArmDisplay::drawOrigin(QPainter& p)
{
	p.save();
	QBrush br(QColor(230, 0, 230));
	p.setBrush(br);
	QPen pn(QColor(230, 0, 230));
	p.setPen(pn);


	QRectF r(0.0, 0.0, 0.2, 0.2);
	p.drawEllipse(r);

	QPoint txtpt = p.transform().map(QPoint(0, 0));
	p.setTransform(QTransform());

	pn = QPen(QColor(0, 0, 0));
	p.setPen(pn);

	p.restore();
}

void ArmDisplay::drawBumpers(QPainter& p)
{
	p.save();

	QBrush br(QColor(255, 0, 0));
	p.setBrush(br);

	QPen pn(QColor(0, 0, 0));
	pn.setWidthF(0.2);
	p.setPen(pn);

	QRectF r(model_.bumperPos().getX(), model_.bumperPos().getY(), model_.bumperSize().getX(), model_.bumperSize().getY());
	p.drawRect(r);

	p.restore();
}

void ArmDisplay::drawTargets(QPainter& p)
{
	p.save();

	QPen pn(QColor(0, 0, 0));
	pn.setWidthF(TargetPenSize);
	p.setPen(pn);

	for (const Translation2d& t : model_.targets()) {
		QPointF p1 = QPoint(t.getX() - TargetXSize, t.getY());
		QPointF p2 = QPoint(t.getX() + TargetXSize, t.getY());
		p.drawLine(p1, p2);

		p1 = QPoint(t.getX(), t.getY() - TargetXSize);
		p2 = QPoint(t.getX(), t.getY() + TargetXSize);
		p.drawLine(p1, p2);
	}

	p.restore();
}


void ArmDisplay::drawJoint(QPainter& p, const QPointF& pt)
{
	QPen bkpen(QColor(0, 0, 0));
	QBrush bkbrush(QColor(0, 0, 0));

	p.save();

	p.setPen(bkpen);
	p.setBrush(bkbrush);
	QRectF rect(pt.x() - JointDotSize / 2.0, pt.y() - JointDotSize / 2.0, JointDotSize, JointDotSize);
	p.drawEllipse(rect);
	p.restore();
}

void ArmDisplay::drawArms(QPainter &p)
{
	QPointF start(model_.armPos().getX(), model_.armPos().getY());
	double baseangle = 0.0;
	QPen pen;

	if (model_.jointCount() > 0) {
		for (int i = 0; i < model_.jointCount(); i++) {
			double angle = baseangle + model_.jointModel(i).angle() * M_PI / 180.0;
			double length = model_.jointModel(i).length();

			pen = QPen(colors_[i]);
			pen.setCapStyle(Qt::RoundCap);
			pen.setWidth(4);
			p.setPen(pen);

			QPointF end = QPointF(start.x() + std::cos(angle) * length, start.y() + std::sin(angle) * length);
			p.drawLine(start, end);

			drawJoint(p, start);

			start = end;
			baseangle = angle;
		}

		drawJoint(p, start);
	}
}
