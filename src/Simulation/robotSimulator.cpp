#include "Simulation/robotSimulator.h"

#include <cmath>

RobotSimulator::RobotSimulator() {
	physicsTimer.clear();
	lastUpdateTime = 0;
}

void RobotSimulator::updatePhysics() {
	// Get delta time
	double currentTime = physicsTimer.value();
	double deltaTime = currentTime - lastUpdateTime;
	lastUpdateTime = currentTime;

	// Update linear motions
	position += velocity * deltaTime;
	velocity += acceleration * deltaTime;
	acceleration += jerk * deltaTime;

	// Update angular motions
	angularPosition += angularVelocity * deltaTime;
	angularVelocity += angularAcceleration * deltaTime;
	angularAcceleration += angularJerk * deltaTime;
}
