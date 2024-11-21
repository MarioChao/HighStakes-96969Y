#pragma once

#include "Simulation/vector3.h"
#include "vex.h"

class RobotSimulator {
public:
	RobotSimulator();

	void resetTimer();
	void updatePhysics();
 
	Vector3 position, velocity, acceleration, jerk;
	double angularPosition, angularVelocity, angularAcceleration, angularJerk;

private:
	timer physicsTimer;
	double lastUpdateTime = 0;
};
