#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds new blue-down autonomous.
void autonpaths::runAutonBlueDown() {
	// Mirrored & modified from red down

	timer autontimer;
	setRotation(75.0);

	// Grab middle goal
	driveAndTurnDistanceTiles(-1.55, 70.0, 100.0, 50.0, defaultMoveTilesErrorRange, 2.0);
	driveAndTurnDistanceTiles(-0.55, 57.0, 80.0, 40.0, defaultMoveTilesErrorRange, 1.0);
	setGoalClampState(1);
	task::sleep(200);

	// Score preload
	setIntakeState(1);
	turnToAngle(75.0, halfRobotLengthIn * 1.3);

	// Take in ring to arm
	turnToAngleVelocity(123, 80.0, 0.0, defaultTurnAngleErrorRange, 2.0);
	setIntakeHookMode(1);
	setIntakeTopState(0);
	driveAndTurnDistanceTiles(0.4, 123.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Drop goal near our side
	turnToAngle(90.0);
	setGoalClampState(0, 0.2);
	driveAndTurnDistanceTiles(1.3, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab goal
	turnToAngle(130.0);
	setIntakeState(-1);
	setIntakeState(0, 0.1);
	setGoalClampState(1, 0.85);
	driveAndTurnDistanceTiles(-1.25, 130.0, 60.0, 50.0, defaultMoveTilesErrorRange, 1.5);

	// Take in corner
	setIntakeHookMode(0);
	setIntakeState(1);
	setArmHangState(1);
	turnToAngle(135.0);
	driveAndTurnDistanceTiles(2.2, 135.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.3);
	task::sleep(200);
	driveAndTurnDistanceTiles(1.0, 135.0, 40.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	task::sleep(400);

	// Score on alliance wall stake
	driveAndTurnDistanceTiles(-2.00, 170.0, 60.0, 50.0, defaultMoveTilesErrorRange, 2.0);
	turnToAngle(57.0);
	driveAndTurnDistanceTiles(0.5, 57.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	setIntakeState(0);
	setArmHangState(0);
	task::sleep(100);

	// Touch ladder
	turnToAngle(60.0);
	driveAndTurnDistanceTiles(-2.0, 100.0, 100.0, 25.0, defaultMoveTilesErrorRange, 2.0);
	turnToAngle(115.0);
}
