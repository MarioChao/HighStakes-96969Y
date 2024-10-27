#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds new blue-up autonomous.
void autonpaths::runAutonBlueUp() {
	// Mirrored from runAutonRedUpNew()

	timer autontimer;
	setRotation(120.0);

	// Score Preload
	setArmHangState(1);
	task::sleep(700);
	driveAndTurnDistanceTiles(0.45, 120.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setArmHangState(0);
	task::sleep(200);

	// Grab goal
	setGoalClampState(1, 1.4);
	driveAndTurnDistanceTiles(-1.70, 115.0, 38.0, 15.0, defaultMoveTilesErrorRange, 2.0);
	// turnToAngle(-80.0, -halfRobotLengthIn * 1.0);
	task::sleep(200);

	// Intake middle up
	setIntakeState(1);
	turnToAngle(-42.0);
	driveAndTurnDistanceTiles(0.92, -42.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-0.10, -42.0, 50.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake 2nd middle up
	turnToAngle(0.0);
	turnToAngleVelocity(-45.0, 80.0, -halfRobotLengthIn * 1.1);

	// Intake right up
	turnToAngle(92.0);
	driveAndTurnDistanceTiles(1.3, 92.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake corner
	setArmHangState(1);
	turnToAngle(50.0);
	driveAndTurnDistanceTiles(1.65, 50.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	driveAndTurnDistanceTiles(0.3, 50.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.2);
	task::sleep(200);
	driveAndTurnDistanceTiles(-0.50, 50.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Touch ladder
	turnToAngle(-120.0);
	while (autontimer.value() < 13.0) {
		task::sleep(20);
	}
	setArmHangState(0);
	setIntakeState(0, 0.6);
	driveAndTurnDistanceTiles(2.2, -120.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(0.5, -120.0, 50.0, 100.0, defaultMoveTilesErrorRange, 1.0);
}
