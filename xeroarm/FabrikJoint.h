#pragma once
class FabrikJoint
{
public:
	enum class ConstraintCoordinateSystem { 
		LOCAL, 
		GLOBAL 
	};

public:
	FabrikJoint() {
		cord_sys_ = ConstraintCoordinateSystem::GLOBAL;
		clock_wise_constraint_ = 0.0;
		counter_clock_wise_constraint_ = 0.0;
	}

	FabrikJoint(ConstraintCoordinateSystem coord, double clock, double counter) {
		cord_sys_ = coord;
		clock_wise_constraint_ = clock;
		counter_clock_wise_constraint_ = counter;
	}

	ConstraintCoordinateSystem coordSystem() const {
		return cord_sys_;
	}

	double clockWise() const {
		return clock_wise_constraint_;
	}

	double counterClockWise() const {
		return counter_clock_wise_constraint_;
	}

	void set(const FabrikJoint& j) {
		cord_sys_ = j.coordSystem();
		clock_wise_constraint_ = j.clockWise();
		counter_clock_wise_constraint_ = j.counterClockWise();
	}

private:
	ConstraintCoordinateSystem cord_sys_;
	double clock_wise_constraint_;
	double counter_clock_wise_constraint_;
};

