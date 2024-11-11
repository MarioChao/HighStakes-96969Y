#pragma once

#include "vex.h"

class MotionProfile {
public:
	MotionProfile();
	MotionProfile(double accelTime, double middleTime, double decelTime);
	void setModeAcceleration(double startAccel, double endDecel, double maxVelocity);
	void setModeTime(double accelTime, double middleTime, double decelTime);
	void setReturnMultipler(double ratio);
	void createProfile(double distance);

	void start();
	double getDistanceAtTime(double currentTimeSeconds);
	double getDistance();
	double getNextSectionDistance();
	double getVelocity();
	double getMotionEndTime();
	bool isDone();
private:
	void createFromMovementDetails(double distance);
	void createFromElapsedTime(double distance);

	timer motionTimer;
	double targetStartTime, targetEndTime, motionEndTime; // "Target" refers to the duration of maximum velocity
	double startAcceleration, endDeceleration, velocityCap;
	double returnRatio;
	int motionSign;
	int calculateMode;
};
