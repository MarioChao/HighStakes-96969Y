#pragma once

#include "Cosmetics/Simulation/vector3.h"
#include "vex.h"

class RobotSimulator {
public:
	RobotSimulator();

	void resetTimer();

	void constrainMotion(double maxVelocity, double trackWidth);
	void updatePhysics();

	void setDistance(double distance);
	void updateDistance();

	void setForwardVelocity(double velocity_value);

	Vector3 position, velocity, acceleration, jerk;

	// Base unit: radians
	double angularPosition, angularVelocity, angularAcceleration, angularJerk;

	double travelledDistance;

private:
	timer physicsTimer;
	Vector3 previousPosition;
	double lastUpdateTime = 0;
};
