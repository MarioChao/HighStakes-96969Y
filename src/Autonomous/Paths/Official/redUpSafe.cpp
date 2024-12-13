#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds new red-up safe autonomous.
void autonpaths::runAutonRedUpSafe() {
	// Modified from red up

	timer autontimer;
	setRobotRotation(-120.0);

	// Grab goal
	setGoalClampState(1, 1.4);
	driveAndTurnDistanceTiles(-1.32, -115.0, 38.0, 15.0, 2.0);
	task::sleep(200);

	// Take in middle up
	turnToAngle(44.0);
	setIntakeState(1);
	driveAndTurnDistanceTiles(0.85, 44.0, 60.0, 100.0, 1.5);
	driveAndTurnDistanceTiles(-0.12, 44.0, 50.0, 100.0, 1.5);

	// Take in 2nd middle up
	turnToAngleVelocity(5.0, 40.0);
	turnToAngleVelocity(40.0, 90.0, halfRobotLengthIn * 1.25);
	task::sleep(100);

	// Take in left up
	turnToAngleVelocity(-70.0, 40.0);
	driveAndTurnDistanceTiles(1.3, -80.0, 60.0, 100.0, 1.5);

	// Take in corner
	// setArmHangState(1);
	setArmStage(2);
	turnToAngle(-50.0);
	setIntakeState(0);
	driveAndTurnDistanceTiles(0.95, -50.0, 80.0, 100.0, 1.0);
	task::sleep(200);
	driveAndTurnDistanceTiles(1.0, -45.0, 40.0, 100.0, 0.5);
	task::sleep(600);
	driveAndTurnDistanceTiles(-0.50, -42.0, 60.0, 100.0, 1.0);

	// Touch ladder
	turnToAngle(-42.0);
	while (autontimer.value() < 13.0) {
		task::sleep(20);
	}
	// setArmHangState(0, 0.5);
	setArmStage(0);
	setIntakeState(0, 0.5);
	driveAndTurnDistanceTiles(-2.24, -42.0, 100.0, 100.0, 1.6);
}
