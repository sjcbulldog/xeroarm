#include "Pose2dTrajectory.h"

Pose2dTrajectory::Pose2dTrajectory()
{
}

Pose2dTrajectory::Pose2dTrajectory(double x, double y) : Pose2d(x,y)
{
}

Pose2dTrajectory::Pose2dTrajectory(double x, double y, const Rotation2d& rot) : Pose2d(x, y, rot)
{
}

Pose2dTrajectory::Pose2dTrajectory(const Translation2d& pos) : Pose2d(pos)
{
}

Pose2dTrajectory::Pose2dTrajectory(const Rotation2d& rot) : Pose2d(rot)
{
}

Pose2dTrajectory::Pose2dTrajectory(const Translation2d& pos, const Rotation2d& rot) : Pose2d(pos, rot)
{
}

Pose2dTrajectory::Pose2dTrajectory(const Pose2d& other) : Pose2d(other)
{
}

Pose2dTrajectory::~Pose2dTrajectory()
{
}

Pose2dTrajectory::Pose2dTrajectory(double x, double y, const Rotation2d& rot, double time, double dist, double vel, double accel) : Pose2d(x, y, rot)
{
	position_ = dist;
	velocity_ = vel;
	acceleration_ = accel;
}

Pose2dTrajectory::Pose2dTrajectory(const Translation2d& pos, const Rotation2d& rot, double time, double dist, double vel, double accel) : Pose2d(pos, rot)
{
	position_ = dist;
	velocity_ = vel;
	acceleration_ = accel;
}

Pose2dTrajectory::Pose2dTrajectory(const Pose2d& other, double time, double dist, double vel, double accel) : Pose2d(other)
{
	position_ = dist;
	velocity_ = vel;
	acceleration_ = accel;
}
