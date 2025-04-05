#include "Cosmetics/Simulation/robotSimulator.h"

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

void RobotSimulator::setForwardDifferentialVoltage(double left_volt, double right_volt, double kv, double tau, double trackWidth) {
	double deltaTime = physicsTimer.time(sec) - lastUpdateTime;
	leftVelocity += (kv * left_volt - leftVelocity) / tau * deltaTime;
	rightVelocity += (kv * right_volt - rightVelocity) / tau * deltaTime;

	double newLinearVelocity = (leftVelocity + rightVelocity) / 2;
	double newAngularVelocity = (rightVelocity - leftVelocity) / 2 / (trackWidth / 2);

	double theta = angularPosition;
	this->velocity = Vector3(newLinearVelocity * cos(theta), newLinearVelocity * sin(theta));
	this->angularVelocity = newAngularVelocity;
}

void RobotSimulator::setForwardDifferentialMotion(double linearVelocity, double angularVelocity, double maxVelocity, double maxAcceleration, double trackWidth) {
	double leftVelocity = linearVelocity - angularVelocity * trackWidth / 2;
	double rightVelocity = linearVelocity + angularVelocity * trackWidth / 2;
	double scaleFactor = aespa_lib::genutil::getScaleFactor(maxVelocity, {leftVelocity, rightVelocity});
	leftVelocity *= scaleFactor;
	rightVelocity *= scaleFactor;

	double prevLinearVelocity = getForwardVelocity();
	double prevAngularVelocity = this->angularVelocity;
	double prevLeftVelocity = prevLinearVelocity - prevAngularVelocity * trackWidth / 2;
	double prevRightVelocity = prevLinearVelocity + prevAngularVelocity * trackWidth / 2;

	double deltaTime = physicsTimer.time(sec) - lastUpdateTime;
	double maxDeltaVelocity = maxAcceleration * deltaTime;
	if (maxAcceleration < 0) maxDeltaVelocity = 1e6;

	leftVelocity = prevLeftVelocity + aespa_lib::genutil::clamp(leftVelocity - prevLeftVelocity, -maxDeltaVelocity, maxDeltaVelocity);
	rightVelocity = prevRightVelocity + aespa_lib::genutil::clamp(rightVelocity - prevRightVelocity, -maxDeltaVelocity, maxDeltaVelocity);

	double newLinearVelocity = (leftVelocity + rightVelocity) / 2;
	double newAngularVelocity = (rightVelocity - leftVelocity) / 2 / (trackWidth / 2);

	double theta = angularPosition;
	this->velocity = Vector3(newLinearVelocity * cos(theta), newLinearVelocity * sin(theta));
	this->angularVelocity = newAngularVelocity;
}

double RobotSimulator::getForwardVelocity() {
	double forwardAngle = atan2(velocity.y, velocity.x);
	double forwardVelocity = velocity.getMagnitude() * cos(forwardAngle - angularPosition);
	return forwardVelocity;
}

aespa_lib::datas::Linegular RobotSimulator::getLookPose() {
	return aespa_lib::datas::Linegular(position.x, position.y, aespa_lib::genutil::toDegrees(angularPosition));
}
