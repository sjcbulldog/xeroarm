#include "FabrikIK.h"
#include "RobotArm.h"

FabrikIK::FabrikIK(const RobotArm& arm) : arm_(arm)
{

}

FabrikChain *FabrikIK::buildChain() const
{
	FabrikChain* chain = new FabrikChain();

	Translation2d start(arm_.pos());
	Translation2d end;
	QVector<double> angles = arm_.angles();

	for (int i = 0; i < arm_.count(); i++)
	{
		end = arm_.jointStartPos(i, angles);
		FabrikJoint joint;

		FabrikBone bone(start, end);
		chain->add(joint, bone);

		start = end;
	}

	return chain;
}

QVector<double> FabrikIK::inverseKinematics(const Translation2d& pt) const
{
	QVector<double> ret;
	FabrikChain* chain = buildChain();

	return ret;
}
