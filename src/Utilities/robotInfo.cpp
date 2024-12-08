#include "Utilities/robotInfo.h"
#include "main.h"

namespace botinfo {
	// Robot info
	const double robotLengthHoles = 27.0; // Left wheel to right wheel
	const double robotLengthIn = robotLengthHoles * (1.0 / 2.0);
	const double halfRobotLengthIn = robotLengthIn / 2;

	const double driveWheelDiameterIn = 4;
	const double driveWheelCircumIn = M_PI * driveWheelDiameterIn;
	const double driveWheelMotorGearRatio = (84.0 / 60.0); // Wheel to Motor

	const double trackingLookWheelDiameterIn = 2.00; // Look wheel -> spins in the forward/backward direction
	const double trackingLookWheelCircumIn = M_PI * trackingLookWheelDiameterIn;
	const double trackingLookWheelSensorGearRatio = 1.0; // Wheel to Encoder / Rotation

	const double chassisMotorRpm = 600.0;
}
