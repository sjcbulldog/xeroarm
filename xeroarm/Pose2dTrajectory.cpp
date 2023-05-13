#include "Pose2dTrajectory.h"

Pose2dTrajectory::Pose2dTrajectory(int size)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(int size, double x, double y) : Pose2d(x,y)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(int size, double x, double y, const Rotation2d& rot) : Pose2d(x, y, rot)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(int size, const Translation2d& pos) : Pose2d(pos)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(int size, const Rotation2d& rot) : Pose2d(rot)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(int size, const Translation2d& pos, const Rotation2d& rot) : Pose2d(pos, rot)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(int size, const Pose2d& other) : Pose2d(other)
{
	init(size);
}

Pose2dTrajectory::Pose2dTrajectory(const Pose2dTrajectory& other, double time, double pos, double vel, double accel)
{
	*this = other;
	time_ = time;
	vel_ = vel;
	pos_ = pos;
	accel_ = accel;
}

Pose2dTrajectory::~Pose2dTrajectory()
{
}

void Pose2dTrajectory::init(int size)
{
	time_ = 0.0;
	aaccel_.resize(size);
	angles_.resize(size);
	avelocity_.resize(size);
}

QVector<double> Pose2dTrajectory::interpolate(const QVector<double>& first, const QVector<double>& second, double pcnt) const
{
	assert(first.count() == second.count());

	QVector<double> ret;

	for (int i = 0; i < first.count(); i++) {
		double v = first.at(i) + (second.at(i) - first.at(i)) * pcnt;
		ret.push_back(v);
	}

	return ret;
}

Pose2dTrajectory Pose2dTrajectory::interpolate(const Pose2dTrajectory& other, double pcnt) const
{
	Pose2d mid = Pose2d::interpolate(other, pcnt);

	double tm = time() + (other.time() - time()) * pcnt;
	double pos = position() + (other.position() - position()) * pcnt;
	double vel = velocity() + (other.velocity() - velocity()) * pcnt;
	double acc = accel() + (other.accel() - accel()) * pcnt;

	QVector<double> nangles = interpolate(angles(), other.angles(), pcnt);
	QVector<double> nvelocity = interpolate(velocities(), other.velocities(), pcnt);
	QVector<double> naaccel = interpolate(aAccel(), other.aAccel(), pcnt);

	Pose2dTrajectory t(nangles.size(), mid);
	t.setAngles(nangles);
	t.setVelocities(nvelocity);
	t.setAaccel(naaccel);

	return Pose2dTrajectory(t, tm, pos, vel, acc);
}