#pragma once

enum class ChangeType
{
	AddJoint,
	UpdateJoint,
	InitialAngle,
	CurrentAngle,
	ArmLength,
	MaxVelocity,
	MaxAccel,
	BumperPos,
	BumperSize,
	ArmPos,
	ArmKeepout,
	Targets,
	AddPath,
	RemovePath,
	RenamePath,
	PathPoint
};
