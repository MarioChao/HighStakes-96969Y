#pragma once

namespace botinfo {
	// Robot info
	extern const double robotLengthHoles;
	extern const double driveWheelDiameterIn;
	extern const double driveWheelMotorGearRatio;
	extern const double trackingLookWheelDiameterIn;
	extern const double chassisMotorRpm;

	// Dervied info
	extern const double robotLengthIn;
	extern const double halfRobotLengthIn; // Used for off-center rotation

	extern const double driveWheelCircumIn;

	extern const double trackingLookWheelCircumIn;
	extern const double trackingLookWheelSensorGearRatio;

	extern const double maxV_tilesPerSec;
}
