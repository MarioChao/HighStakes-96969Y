#include "AutonUtilities/motionProfile.h"

// Public

/// @brief Default constructor for a motion profile object.
MotionProfile::MotionProfile() {
	setModeTime(0.1, 0.3, 0.1);
	returnRatio = 1;
	motionSign = 0;
}

/// @brief Construct a motion profile object using elapsed time.
/// @param accelTime The duration of the acceleration phase.
/// @param middleTime The duration of the max-velocity phase.
/// @param decelTime The duration of the deceleration phase.
MotionProfile::MotionProfile(double accelTime, double middleTime, double decelTime) {
	setModeTime(accelTime, middleTime, decelTime);
	returnRatio = 1;
	motionSign = 0;
}

/// @brief Set the motion profile to be calculated from movement details.
/// @param startAccel The magnitude of acceleration at the start (positive).
/// @param endDecel The magnitude of deceleration at the end (positive).
/// @param maxVelocity The maximum velocity of the motion profile (positive).
void MotionProfile::setModeAcceleration(double startAccel, double endDecel, double maxVelocity) {
	startAcceleration = fmax(0.05, fabs(startAccel));
	endDeceleration = -fmax(0.05, fabs(endDecel));
	velocityCap = fabs(maxVelocity);
	if (fabs(velocityCap) <= 1e-9) {
		velocityCap = 0.05;
	}

	targetStartTime = targetEndTime = motionEndTime = 0;
	calculateMode = 1;
}

/// @brief Set the motion profile to be calculated from elapsed time.
/// @param accelTime The duration of the acceleration phase.
/// @param middleTime The duration of the max-velocity phase.
/// @param decelTime The duration of the deceleration phase.
void MotionProfile::setModeTime(double accelTime, double middleTime, double decelTime) {
	targetStartTime = fabs(accelTime);
	targetEndTime = targetStartTime + fabs(middleTime);
	motionEndTime = targetEndTime + fabs(decelTime);
	if (fabs(targetStartTime + targetEndTime + motionEndTime) < 1e-9) {
		targetEndTime = motionEndTime = 1;
	}

	startAcceleration = endDeceleration = velocityCap = 0;
	calculateMode = 0;
}

void MotionProfile::setReturnMultipler(double ratio) {
	returnRatio = ratio;
}

/// @brief Calculate the needed values for the motion profile to travel a given distance.
/// @param distance The desired distance to travel.
void MotionProfile::createProfile(double distance) {
	switch (calculateMode) {
		case 0:
			createFromElapsedTime(distance);
			break;
		case 1:
			createFromMovementDetails(distance);
			break;
	}
	motionSign = (distance > 0) - (distance < 0);
	// printf("TargetSt: %.3f, TargetEd: %.3f, MotionEnd: %.3f\n", targetStartTime, targetEndTime, motionEndTime);
}

void MotionProfile::start() {
	motionTimer.reset();
}

double MotionProfile::getDistanceAtTime(double timeSeconds) {
	double distance = 0;
	if (timeSeconds < targetStartTime) {
		distance += startAcceleration * pow(timeSeconds, 2) / 2;

	} else if (timeSeconds < targetEndTime) {
		distance += startAcceleration * pow(targetStartTime, 2) / 2;
		distance += (startAcceleration * targetStartTime) * (timeSeconds - targetStartTime);

	} else if (timeSeconds < motionEndTime) {
		distance += startAcceleration * pow(targetStartTime, 2) / 2;
		distance += (startAcceleration * targetStartTime) * (targetEndTime - targetStartTime);
		distance += (startAcceleration * targetStartTime) * (timeSeconds - targetEndTime);
		distance += endDeceleration * pow(timeSeconds - targetEndTime, 2) / 2;

	} else {
		distance += startAcceleration * pow(targetStartTime, 2) / 2;
		distance += (startAcceleration * targetStartTime) * (targetEndTime - targetStartTime);
		distance += (startAcceleration * targetStartTime) * (motionEndTime - targetEndTime);
		distance += endDeceleration * pow(motionEndTime - targetEndTime, 2) / 2;

	}
	return distance *= motionSign;
}
double MotionProfile::getDistance() {
	return getDistanceAtTime(motionTimer.value());
}
double MotionProfile::getNextSectionDistance() {
	double currentTimeSeconds = motionTimer.value();
	if (currentTimeSeconds < targetStartTime) {
		return getDistanceAtTime(targetStartTime);
	} else if (currentTimeSeconds < targetEndTime) {
		return getDistanceAtTime(targetEndTime);
	} else {
		return getDistanceAtTime(motionEndTime);
	}
}
double MotionProfile::getVelocity() {
	double currentTime = motionTimer.value();
	double velocity;
	if (currentTime < targetStartTime) {
		velocity = startAcceleration * currentTime;
	} else if (currentTime < targetEndTime) {
		velocity = startAcceleration * targetStartTime;
	} else if (currentTime < motionEndTime) {
		velocity = -endDeceleration * (motionEndTime - currentTime);
	} else {
		velocity = 0;
	}
	return velocity * returnRatio * motionSign;
}

double MotionProfile::getMotionEndTime() {
	return motionEndTime;
}

bool MotionProfile::isDone() {
	return motionTimer.value() > motionEndTime;
}

// Private

/// @brief Calculate and set the elapsed time for the motion profile.
/// @param distance The traveled distance of the motion profile.
void MotionProfile::createFromMovementDetails(double distance) {
	/* Kinematics:
		vf^2 = vi^2 + 2a * Δx
		Δx = (vf^2 - vi^2) / (2a)
	*/
	// Distance (trapezoidal):
	// x_start = v_max^2 / (2 * a_start)
	// x_middle = v_max * t
	// x_end = -v_max^2 / (2 * a_end)
	double absDistance = fabs(distance);
	double distanceStart = pow(velocityCap, 2) / (2 * startAcceleration);
	double distanceEnd = -pow(velocityCap, 2) / (2 * endDeceleration);
	if (distanceStart + distanceEnd >= absDistance) {
		// Triangular motion
		// Velocity:
		// x_total = v_max^2 * (a_end - a_start) / (2 * a_start * a_end)
		// v_max = √(x_total * (2 * a_start * a_end) / (a_end - a_start))
		double velocityPeak = sqrt(absDistance * (2 * startAcceleration * endDeceleration) / (endDeceleration - startAcceleration));
		// Time:
		// v = at
		// t = v / a
		targetStartTime = targetEndTime = velocityPeak / startAcceleration;
		motionEndTime = targetEndTime + (-velocityPeak / endDeceleration);
		printf("Triangular motion profile.\n");
	} else {
		// Trapezoidal motion
		targetStartTime = velocityCap / startAcceleration;
		targetEndTime = targetStartTime + (absDistance - (distanceStart + distanceEnd)) / velocityCap;
		motionEndTime = targetEndTime + (-velocityCap / endDeceleration);
		printf("Trapezoidal motion profile.\n");
		// printf("D1: %.3f, D2: %.3f, D3: %.3f\n", distanceStart, absDistance - (distanceStart + distanceEnd), distanceEnd);
		// printf("A1: %.3f, MV: %.3f, A3: %.3f\n", startAcceleration, velocityCap, endDeceleration);

	}
}

/// @brief Calculate and set the acceleration and maximum velocity for the motion profile.
/// @param distance The traveled distance of the motion profile.
void MotionProfile::createFromElapsedTime(double distance) {
	// Distance (trapezoidal):
	// x_start = v_max * t_1 / 2
	// x_middle = v_max * t_2
	// x_end = v_max * t_3 / 2
	// x_total = v_max * (t_1 / 2 + t_2 + t_3 / 2)
	// Velocity:
	// v_max = x_total / (t_1 / 2 + t_2 + t_3 / 2)
	double accelTime = targetStartTime;
	double middleTime = targetEndTime - targetStartTime;
	double decelTime = motionEndTime - targetEndTime;
	velocityCap = fabs(distance) / (accelTime / 2.0 + middleTime + decelTime / 2.0);
	// Acceleration:
	// vf = vi + at
	// a = (vf - vi) / t
	startAcceleration = velocityCap / accelTime;
	endDeceleration = -velocityCap / decelTime;
}
