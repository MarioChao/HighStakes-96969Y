#pragma once

#include "Simulation/vector3.h"
#include "vex.h"

class RobotSimulator {
public:
	RobotSimulator();

	void resetTimer();
	void updatePhysics();

	void setDistance(double distance);
	void updateDistance();

	Vector3 position, velocity, acceleration, jerk;
	double angularPosition, angularVelocity, angularAcceleration, angularJerk;

	double travelledDistance;

private:
	timer physicsTimer;
	Vector3 previousPosition;
	double lastUpdateTime = 0;
};
