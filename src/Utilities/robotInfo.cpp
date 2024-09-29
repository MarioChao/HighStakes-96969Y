#include "Utilities/robotInfo.h"
#include "main.h"

namespace botinfo {
	// Robot info
	double robotLengthHoles = 27.0; // Left wheel to right wheel
	double robotLengthIn = robotLengthHoles * (1.0 / 2.0);
	double halfRobotLengthIn = robotLengthIn / 2; // Used for off-center rotation

	double driveWheelDiameterIn = 3.25;
	double driveWheelCircumIn = M_PI * driveWheelDiameterIn;
	double driveWheelMotorGearRatio = (60.0 / 48.0); // Wheel to Motor

	// Velocity from pct to rev/s
	// 100% = 600/60 rev/s
	// 100.0 * 60.0 / 600.0 (%s/rev) = 1
	double driveMotorRevToPercentSecFactor = 100.0 * 60.0 / 600.0; // Used for motion profile auton (not used for now)

	double trackingLookWheelDiameterIn = 2.00; // Look wheel -> spins in the forward/backward direction
	double trackingLookWheelCircumIn = M_PI * trackingLookWheelDiameterIn;
	double trackingLookWheelSensorGearRatio = 1.0; // Wheel to Encoder / Rotation
}
