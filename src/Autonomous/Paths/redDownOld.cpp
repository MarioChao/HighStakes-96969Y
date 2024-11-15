#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds old red-down autonomous.
void autonpaths::runAutonRedDownOld() {
	timer autontimer;
	setRotation(-90.0);

	// Grab middle goal
	driveAndTurnDistanceTiles(-1.30, -90.0, 100.0);
	turnToAngle(-120.0, halfRobotLengthIn * 0.5);
	setGoalClampState(1, 0.7);
	driveAndTurnDistanceTiles(-0.74, -120.0, 60.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	// Score 1 ring
	task::sleep(200);
	setIntakeState(1);
	task::sleep(700);

	// Drop goal & face bottom ring
	turnToAngleVelocity(-50, 50.0, halfRobotLengthIn * 0.85);
	setGoalClampState(0);
	setIntakeState(0);

	// Intake bottom ring
	setIntakeTopState(0);
	setIntakeBottomState(1);
	setIntakeBottomState(0, 0.3);
	driveAndTurnDistanceTiles(0.43, -50, 60.0, 100.0);

	// Grab bottom goal
	turnToAngle(-180.0);
	driveAndTurnDistanceTiles(-0.75, -180.0, 30.0);
	setGoalClampState(1);

	// Score 1 ring
	setIntakeState(1);
	task::sleep(750);

	// Face the ladder
	turnToAngle(-270.0);
	setIntakeState(0);

	// Touch the ladder
	driveAndTurnDistanceTiles(0.9, -270.0, 30.0, 100.0);
}
