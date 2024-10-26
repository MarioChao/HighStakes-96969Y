#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds new red-up autonomous.
void autonpaths::runAutonRedUpNew() {
	timer autontimer;
	setRotation(-120.0);

	// Score Preload
	setArmHangState(1);
	task::sleep(700);
	driveAndTurnDistanceTiles(0.45, -120.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setArmHangState(0);
	task::sleep(200);

	// Grab goal
	setGoalClampState(1, 1.4);
	driveAndTurnDistanceTiles(-1.75, -110.0, 50.0, 15.0, defaultMoveTilesErrorRange, 2.0);
	// turnToAngle(-80.0, -halfRobotLengthIn * 1.0);
	task::sleep(200);

	// Intake middle up
	setIntakeState(1);
	turnToAngle(40.0);
	driveAndTurnDistanceTiles(0.95, 40.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-0.16, 40.0, 50.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake 2nd middle up
	turnToAngle(0.0);
	turnToAngleVelocity(35.0, 80.0, halfRobotLengthIn * 1.37);
	// turnToAngleVelocity(0.0, 80.0);

	// Intake left up
	turnToAngle(-90.0);
	driveAndTurnDistanceTiles(1.0, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake corner
	setArmHangState(1);
	turnToAngle(-58.0);
	driveAndTurnDistanceTiles(1.65, -58.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	driveAndTurnDistanceTiles(0.3, -58.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.2);
	task::sleep(200);
	driveAndTurnDistanceTiles(-0.50, -58.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Touch ladder
	turnToAngle(120.0);
	while (autontimer.value() < 13.0) {
		task::sleep(20);
	}
	setArmHangState(0);
	setIntakeState(0, 0.6);
	driveAndTurnDistanceTiles(2.0, 120.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(0.5, 120.0, 50.0, 100.0, defaultMoveTilesErrorRange, 1.0);
}

/// @brief Run the 15-seconds red-up autonomous.
void autonpaths::runAutonRedUp() {
	timer autontimer;
	//setGoalClampState(1);
	setGoalClampState(0);
	setRotation(-90.0);

	// Grab goal
	driveAndTurnDistanceTiles(-1.1, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setGoalClampState(1, 0.4);
	driveAndTurnDistanceTiles(-0.5, -90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// turnToAngle(-80.0, -halfRobotLengthIn * 1.0);
	task::sleep(30);

	//driveAndTurnDistanceTiles(-0.3, 105.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	//driveAndTurnDistanceTiles(-0.4, 90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// Intake middle up
	setIntakeState(1);
	//task::sleep(1000);
	turnToAngle(40.0);
	driveAndTurnDistanceTiles(1.06, 40.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	//task::sleep(500);
	driveAndTurnDistanceTiles(-0.16, 40.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(0.0);
	// task::sleep(500);
	driveAndTurnDistanceTiles(0.1, 0.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngleVelocity(65.0, 70.0, halfRobotLengthIn * 1.37);
	// driveAndTurnDistanceTiles(0.6, 0.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// task::sleep(750);
	//turnToAngle(-55.0, -halfRobotLengthIn * 0.75);
	//turnToAngle(0, halfRobotLengthIn * 0.75);
	// Intake up
	turnToAngleVelocity(0.0, 70.0);
	// driveAndTurnDistanceTiles(-0.3, 0.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);


	// Touch ladder
	turnToAngle(-90.0);
	driveAndTurnDistanceTiles(0.6, -90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setIntakeState(0, 0.75);
	driveAndTurnDistanceTiles(0.4, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(125);
	setIntakeState(1);
	// setGoalClampState(0, 0.5);
	// task::sleep(2000);
	while (autontimer.value() < 12.0) {
		task::sleep(20);
	}
	driveAndTurnDistanceTiles(2.0, 125.0, 30.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	//*/
}
