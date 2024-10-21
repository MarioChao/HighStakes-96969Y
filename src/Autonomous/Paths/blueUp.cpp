#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds new blue-up autonomous.
void autonpaths::runAutonBlueUpNew() {
	// Mirrored from runAutonRedUpNew()

	timer autontimer;
	//setGoalClampState(1);
	setGoalClampState(0);
	setRotation(120.0);

	// Score Preload
	setArmHangState(1);
	task::sleep(300);
	driveAndTurnDistanceTiles(0.45, 120.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setArmHangState(0);
	task::sleep(200);

	// Grab goal
	setGoalClampState(1, 1.4);
	driveAndTurnDistanceTiles(-1.75, 110.0, 40.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	task::sleep(200);

	// Intake middle up
	setIntakeState(1);
	turnToAngle(-40.0);
	driveAndTurnDistanceTiles(0.85, -40.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-0.16, -40.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake 2nd middle up
	turnToAngle(0.0);
	driveAndTurnDistanceTiles(0.1, 0.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngleVelocity(-35.0, 70.0, -halfRobotLengthIn * 1.37);
	turnToAngleVelocity(0.0, 70.0);

	// Intake left up
	turnToAngleVelocity(90.0, 70.0);
	driveAndTurnDistanceTiles(1.0, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setIntakeState(0);
	turnToAngle(-125);
	setIntakeState(1);

	while (autontimer.value() < 12.0) {
		task::sleep(20);
	}
	// Touch ladder
	driveAndTurnDistanceTiles(2.0, -125.0, 30.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	//*/
}

/// @brief Run the 15-seconds blue-up autonomous.
void autonpaths::runAutonBlueUp() {
	timer autontimer;
	//setGoalClampState(1);
	setGoalClampState(0);
	setRotation(90.0);

	// Grab goal
	driveAndTurnDistanceTiles(-1.1, 90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setGoalClampState(1, 0.5);
	driveAndTurnDistanceTiles(-0.15, 90.0, 15.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	//driveAndTurnDistanceTiles(-0.3, 105.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	//driveAndTurnDistanceTiles(-0.4, 90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// Intake middle up
	setIntakeState(1);
	// task::sleep(1000);
	turnToAngle(-55.0);
	driveAndTurnDistanceTiles(1.04, -55.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// task::sleep(500);
	driveAndTurnDistanceTiles(-0.18, -55.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(0.0);
	// task::sleep(500);
	driveAndTurnDistanceTiles(0.1, 0.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngleVelocity(-65.0, 70.0, -halfRobotLengthIn * 1.30);
	// driveAndTurnDistanceTiles(0.6, 0.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// task::sleep(750);
	//turnToAngle(-55.0, -halfRobotLengthIn * 0.75);
	//turnToAngle(0, halfRobotLengthIn * 0.75);
	// Intake up
	turnToAngleVelocity(0.0, 70.0);
	// driveAndTurnDistanceTiles(-0.3, 0.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);


	turnToAngle(90.0);
	driveAndTurnDistanceTiles(0.6, 90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setIntakeState(0, 0.75);
	driveAndTurnDistanceTiles(0.4, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(-125);
	setIntakeState(1);
	//setGoalClampState(0, 0.5);
	// Touch ladder
	while (autontimer.value() < 12.0) {
		task::sleep(20);
	}
	driveAndTurnDistanceTiles(2.0, -125.0, 30.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	//*/
}
