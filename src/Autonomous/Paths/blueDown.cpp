#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds blue-down autonomous.
void autonpaths::runAutonBlueDown() {
	timer autontimer;
	setRotation(90.0);


	/* Start facing left */

	// Grab middle goal
	turnToAngle(90.0);
	driveAndTurnDistanceTiles(-1.4, 90.0, 60);
	turnToAngle(118.0, -halfRobotLengthIn * 0.5);
	setGoalClampState(1, 0.85);
	driveAndTurnDistanceTiles(-0.65, 118.0, 30.0, 100.0, defaultMoveTilesErrorRange, 3.0);

	task::sleep(200);
	setIntakeState(1);

	// Score 1 ring
	//turnToAngle(125.0);

	turnToAngleVelocity(120.0, 30.0, halfRobotLengthIn * 0.75);
	driveAndTurnDistanceTiles(0.365, 120.0);

	task::sleep(750);


	// Drop goal
	//setIntakeState(0);
	turnToAngle(50.0);
	setGoalClampState(0);

	// Intake bottom ring
	// turnToAngle(48.5);
	setIntakeTopState(0);
	setIntakeBottomState(1);
	driveAndTurnDistanceTiles(0.6, 50.0, 20.0, 100.0);


	// Grab bottom goal
	turnToAngle(180.0);
	driveAndTurnDistanceTiles(-0.4, 180.0);
	driveAndTurnDistanceTiles(-0.4, 180.0, 20.0);
	setGoalClampState(1);


	// Score 1 ring
	setIntakeState(1);
	task::sleep(750);


	// Drop goal
	turnToAngle(270.0);
	setIntakeState(0);
	setGoalClampState(0);


	// Touch the ladder
	driveAndTurnDistanceTiles(0.9, 270.0, 30.0, 100.0);
}
