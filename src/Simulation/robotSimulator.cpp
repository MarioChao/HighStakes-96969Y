#include "Simulation/robotSimulator.h"

#include <cmath>

RobotSimulator::RobotSimulator() {
	resetTimer();
	travelledDistance = 0;
}

void RobotSimulator::resetTimer() {
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

void RobotSimulator::setDistance(double distance) {
	previousPosition = position;
	travelledDistance = distance;
}

void RobotSimulator::updateDistance() {
	// Update distance
	travelledDistance += (position - previousPosition).getMagnitude();

	// Update previous
	previousPosition = position;
}
