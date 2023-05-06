#pragma once

#include "ArmDataModel.h"
#include "SplinePair.h"

#include <QtWidgets/QWidget>

class ArmDisplay : public QWidget
{
	Q_OBJECT

public:
	ArmDisplay(ArmDataModel &model, QWidget* parent);
	~ArmDisplay();

public:
	void resetDisplay();
	void setCurrentPath(std::shared_ptr<ArmPath> path) {
		path_ = path;
		repaint();
	}

signals:
	void pathPointSelected(std::shared_ptr<ArmPath> path, int index);

protected:
	void paintEvent(QPaintEvent* ev) override ;
	void resizeEvent(QResizeEvent* ev) override;
	void keyPressEvent(QKeyEvent* ev) override;

	void mousePressEvent(QMouseEvent* ev) override;
	void mouseReleaseEvent(QMouseEvent* ev) override;
	void mouseMoveEvent(QMouseEvent* ev) override;

private:
	double armLength();
	QRectF armBounds();
	QRectF bumperBounds();
	QRectF targetBounds();

	void redraw(ChangeType type) {
		repaint(); 
	}

	void calcTransforms();


	void drawArms(QPainter& p);
	void drawBumpers(QPainter& p);
	void drawTargets(QPainter& p);
	void drawOrigin(QPainter& p);
	void drawCurrentPath(QPainter& p);
	void drawSplines(QPainter& p);
	void drawSpline(QPainter& paint, std::shared_ptr<SplinePair> pair);
	void drawPoints(QPainter& p);
	void drawOnePoint(QPainter& paint, const Pose2d& pt, bool selected);
	void drawJoint(QPainter& p, const QPointF& pt);

	void findSplineStep(std::shared_ptr<SplinePair> pair);
	QVector<std::shared_ptr<SplinePair>> computeSplinesForPath(std::shared_ptr<ArmPath> path);

	bool hitTest(const QPointF& pt, int& index, bool& center);

private:
	ArmDataModel& model_;
	QMargins margins_;
	QPoint origin_;
	double scale_;
	std::shared_ptr<ArmPath> path_;
	int selected_;

	QPoint angles_loc_;

	static QVector<QColor> colors_;

	static double constexpr const TargetXSize = 1.0;
	static double constexpr const TargetPenSize = 0.5;
	static double constexpr const OriginSize = 1.0;
	static double constexpr const JointDotSize = 0.5;
	static double constexpr const PathPointSize = 0.25;
	static double constexpr const TriangleSize = 2.0;

	QVector<QPointF> triangle_;
	QTransform xform_;

	bool dragging_;
	bool rotating_;
};

