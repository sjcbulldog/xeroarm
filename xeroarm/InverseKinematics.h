#pragma once

#include "Translation2d.h"
#include <QtCore/QVector>

class InverseKinematics
{
public:
	virtual QVector<double> inverseKinematics(const Translation2d& pt) const = 0;
};

