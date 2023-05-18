#pragma once

#include "InverseKinematics.h"
#include "FabrikChain.h"

class RobotArm;

class FabrikIK : public InverseKinematics
{
public:
	FabrikIK(const RobotArm& arm);
	virtual QVector<double> inverseKinematics(const Translation2d& pt) const ;

private:
	FabrikChain *buildChain() const ;

private:
	const RobotArm& arm_;
};

