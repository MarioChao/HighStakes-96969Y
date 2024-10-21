#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds red-down autonomous.
void autonpaths::runAutonRedDown() {
	timer autontimer;
	setRotation(-90.0);


	/* Start facing left */

	// Grab middle goal
	turnToAngle(-90.0);
	driveAndTurnDistanceTiles(-1.5, -90.0, 60);
	turnToAngle(-118.0, -halfRobotLengthIn * 0.5);
	setGoalClampState(1, 0.92);
	driveAndTurnDistanceTiles(-0.7, -118.0, 30.0, 100.0, defaultMoveTilesErrorRange, 3.0);

	task::sleep(200);
	setIntakeState(1);
	// Score 1 ring
	//turnToAngle(125.0);
	task::sleep(200);
	driveAndTurnDistanceTiles(0.25, -118.0, 40.0);

	task::sleep(500);


	// Drop goal
	//setIntakeState(0);
	turnToAngle(-50);
	setGoalClampState(0);

	// Intake bottom ring
	setIntakeTopState(0);
	setIntakeBottomState(1);
	driveAndTurnDistanceTiles(0.6, -50, 20.0, 100.0);


	// Grab bottom goal
	turnToAngle(-180.0);
	driveAndTurnDistanceTiles(-0.45, -180.0);
	driveAndTurnDistanceTiles(-0.37, -180.0, 20.0);
	setGoalClampState(1);


	// Score 1 ring
	setIntakeState(1);
	task::sleep(750);


	// Drop goal
	turnToAngle(-270.0);
	setIntakeState(0);
	setGoalClampState(0);


	// Touch the ladder
	driveAndTurnDistanceTiles(0.9, -270.0, 30.0, 100.0);
}
