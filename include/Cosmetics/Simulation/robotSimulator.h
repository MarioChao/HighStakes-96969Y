#pragma once

#include "Cosmetics/Simulation/vector3.h"
#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "vex.h"

class RobotSimulator {
public:
	RobotSimulator();

	void resetTimer();

	void constrainMotion(double maxVelocity, double trackWidth);
	void updatePhysics();

	void setDistance(double distance);
	void updateDistance();

	void setForwardDifferentialVoltage(double left_volt, double right_volt, double kv, double tau, double trackWidth);
	void setForwardDifferentialMotion(double linearVelocity, double angularVelocity, double maxVelocity, double maxAcceleration, double trackWidth);
	double getForwardVelocity();

	aespa_lib::datas::Linegular getLookPose();


	// Differential info
	double leftVelocity = 0, rightVelocity = 0;

	// Base unit: any length
	Vector3 position, velocity, acceleration, jerk;

	// Base unit: radians
	double angularPosition, angularVelocity, angularAcceleration, angularJerk;

	double travelledDistance;

private:
	timer physicsTimer;
	Vector3 previousPosition;
	double lastUpdateTime = 0;
};
