#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "main.h"

namespace botinfo {
	// Robot info
	const double robotLengthHoles = 24.0; // Left wheel to right wheel
	const double driveWheelDiameterIn = 2.75;
	const double driveWheelMotorGearRatio = (36.0 / 36.0); // Wheel to Motor (wheel teeth / motor teeth)
	const double chassisMotorRpm = 600.0;
	const double trackingLookWheelDiameterIn = 2.00; // Look wheel -> spins in the forward/backward direction

	// Derived info
	const double robotLengthIn = robotLengthHoles * (1.0 / 2.0);
	const double halfRobotLengthIn = robotLengthIn / 2;

	const double driveWheelCircumIn = M_PI * driveWheelDiameterIn;

	const double driveMotorToWheel_gearRatio = 1.0 / driveWheelMotorGearRatio;

	const double trackingLookWheelCircumIn = M_PI * trackingLookWheelDiameterIn;
	const double trackingLookWheelSensorGearRatio = 1.0; // Wheel to Encoder / Rotation

	const double maxV_tilesPerSec = (
		chassisMotorRpm * (1 / 60.0) * (1.0 / driveWheelMotorGearRatio) // wheel rev/sec
		* (driveWheelCircumIn / 1.0) * (1.0 / field::tileLengthIn) // tiles/sec
	);

	const double tilesPerSecond_to_pct = (
		// input: travel tiles per second
		(1.0 / botinfo::maxV_tilesPerSec) // motor's pct [0-1]
		* (100.0 / 1.0) // motor's pct [0-100]
	);
}
