#pragma once

#include "FabrikBone.h"
#include "FabrikJoint.h"
#include <QtCore/QVector>

class FabrikChain
{
public:
	FabrikChain();

	void clear() {
		joints_.clear();
		bones_.clear();
	}

	bool solveIK(const Translation2d& target);

	void add(const FabrikJoint& j, const FabrikBone& b) {
		joints_.push_back(j);
		bones_.push_back(b);
	}

private:

private:
	QVector<FabrikJoint> joints_;
	QVector<FabrikBone> bones_;
};

