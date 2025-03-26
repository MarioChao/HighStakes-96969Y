#include "Simulation/robotSimulator.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>

RobotSimulator::RobotSimulator() {
	resetTimer();
	travelledDistance = 0;
}

void RobotSimulator::resetTimer() {
	physicsTimer.clear();
	lastUpdateTime = 0;
}

void RobotSimulator::constrainMotion(double maxVelocity, double trackWidth) {
	double tempLinearVelocity = aespa_lib::genutil::clamp(velocity.getMagnitude(), -maxVelocity, maxVelocity);
	if (tempLinearVelocity > 1e-5) {
		double velocityScalar = tempLinearVelocity / velocity.getMagnitude();
		velocity *= velocityScalar;
		// printf("V: %.3f, %.3f, %.3f\n", velocity.x, velocity.y, velocity.z);
	}

	if (trackWidth > 1e-5) {
		tempLinearVelocity = angularVelocity * trackWidth / 2;
		tempLinearVelocity = aespa_lib::genutil::clamp(tempLinearVelocity, -maxVelocity, maxVelocity);
		angularVelocity = tempLinearVelocity / (trackWidth / 2);
	}

	// Scale down both
	double leftVelocity = velocity.getMagnitude() - angularVelocity * trackWidth / 2;
	double rightVelocity = velocity.getMagnitude() + angularVelocity * trackWidth / 2;
	double scaleFactor = aespa_lib::genutil::getScaleFactor(maxVelocity, {leftVelocity, rightVelocity});
	leftVelocity *= scaleFactor;
	rightVelocity *= scaleFactor;
	double linearVelocity = (leftVelocity + rightVelocity) / 2;
	if (velocity.getMagnitude() > 1e-5) {
		double velocityScalar = (linearVelocity) / velocity.getMagnitude();
		velocity *= velocityScalar;
	}
	if (trackWidth > 1e-5) {
		angularVelocity = (rightVelocity - leftVelocity) / 2 / (trackWidth / 2);
	}
}

void RobotSimulator::updatePhysics() {
	// Get delta time
	double currentTime = physicsTimer.time(sec);
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

void RobotSimulator::setForwardVelocity(double velocity_value) {
	double theta = angularPosition;
	velocity = Vector3(velocity_value * cos(theta), velocity_value * sin(theta));
}
