#pragma once

#include "Translation2d.h"
#include "FabrikJoint.h"
#include <QtCore/QString>

class FabrikBone
{
public:
	FabrikBone(const Translation2d& start, const Translation2d& end) {
		start_ = start;
		end_ = end;
		length_ = start_.distance(end_);
	}

	FabrikBone(const Translation2d& start, const Translation2d& dir, double l) {
		start_ = start;
		end_ = start + dir.normal() * l;
		length_ = l;
	}

	double length() const {
		return length_;
	}

	Translation2d getDirectionUV() const {
		return (end_ - start_).normal();
	}

private:
	Translation2d start_;
	Translation2d end_;
	double length_;
};

