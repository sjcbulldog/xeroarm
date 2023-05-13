#include "ArmMotionProfile.h"

Pose2dTrajectory ArmMotionProfile::getByTime(double t)
{
	int low = 0;
	int high = traj_.count() - 1;

	while (high - low > 1) {
		int half = (low + high) / 2;
		if (t < traj_.at(half).time()) {
			high = half;
		}
		else {
			low = half;
		}
	}

	double pcnt = (t - traj_.at(low).time()) / (traj_.at(high).time() - traj_.at(low).time());
	return traj_.at(low).interpolate(traj_.at(high), pcnt);
}