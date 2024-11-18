#include "Autonomous/autonPaths.h"

/// @brief Run the skills autonomous.
void autonpaths::runAutonSkills() {
	timer autontimer;
	setRotation(-90.0);

	// Score preload at alliance wall stake
	setArmHangState(1);
	driveAndTurnDistanceTiles(-0.2, -90.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	task::sleep(200);
	driveAndTurnDistanceTiles(0.35, -90.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setArmHangState(0);
	task::sleep(200);

	// Grab bottom mobile goal
	driveAndTurnDistanceTiles(-0.5, -90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(0);
	setGoalClampState(1, 0.7);
	driveAndTurnDistanceTiles(-0.96, 0, 60.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	// Take in 2 rings & score
	turnToAngle(90.0);
	setIntakeState(1);
	driveAndTurnDistanceTiles(0.85, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	turnToAngle(143.0);
	driveAndTurnDistanceTiles(1.65, 143.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	// Take in 3 rings in a row & score
	driveAndTurnDistanceTiles(-0.40, 143.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	turnToAngle(270.0);
	driveAndTurnDistanceTiles(2.10, 270.0, 60.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	// Take in 1 ring to arm
	turnToAngle(270.0);
	turnToAngle(180.0, halfRobotLengthIn * 1.0);
	setIntakeHookMode(1);
	driveAndTurnDistanceTiles(0.6, 180.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Place mobile goal in corner
	turnToAngle(85.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-0.60, 85.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Go to & face neutral wall stake
	setIntakeState(0);
	setIntakeHookMode(0);
	turnToAngle(85.0);
	setArmHangState(1);
	driveAndTurnDistanceTiles(2.46, 85.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.5);
	turnToAngleVelocity(180.0, 60.0);

	// Score ring at neutral wall stake
	driveAndTurnDistanceTiles(0.7, 180.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setArmHangState(0);
	driveAndTurnDistanceTiles(0.1, 180.0, 50.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	task::sleep(200);

	/* Blue down */

	// Store 1 ring to arm
	turnToAngle(180.0);
	driveAndTurnDistanceTiles(-0.5, 180.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(90.0);
	setIntakeHookMode(1);
	setIntakeState(1);
	driveAndTurnDistanceTiles(1.0, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Store 1 ring
	turnToAngle(0.0);
	setIntakeBottomState(1);
	setIntakeTopState(0);
	driveAndTurnDistanceTiles(1.0, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab bottom mobile goal w/ blue ring
	turnToAngle(-90.0);
	driveAndTurnDistanceTiles(-0.9, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setGoalClampState(1, 0.6);
	driveAndTurnDistanceTiles(-0.6, -90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Put mobile goal in corner
	turnToAngle(-15.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-1.55, -10.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.3);

	/* Blue up */

	// Go to middle mobile goal
	setIntakeState(0);
	driveAndTurnDistanceTiles(0.4, -20.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(-20.0);
	driveAndTurnDistanceTiles(1.28, -20.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab middle mobile goal
	turnToAngle(-180.0);
	setGoalClampState(1, 0.7);
	driveAndTurnDistanceTiles(-0.80, -180.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Score 1 ring at blue wall stake
	setArmHangState(1);
	turnToAngle(-280.0);
	driveAndTurnDistanceTiles(0.53, -270.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	setArmHangState(0);
	task::sleep(400);
	driveAndTurnDistanceTiles(-0.3, -270.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Score 1 stored ring
	setIntakeState(1);

	// Take in 3 rings & score 
	turnToAngle(-270.0);
	setRotation(90.0);
	// First ring
	turnToAngle(-50.0);
	driveAndTurnDistanceTiles(1.86, -50.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	// Second ring
	turnToAngle(-135.0);
	setIntakeTopState(0);
	driveAndTurnDistanceTiles(1.0, -135.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-1.1, -135.0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setIntakeTopState(1);
	// Third ring
	turnToAngle(0);
	driveAndTurnDistanceTiles(1.03, 0, 90.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Take in 2 rings in a row & score
	turnToAngle(90.0);
	driveAndTurnDistanceTiles(1.68, 90.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Take in 1 ring to arm
	turnToAngle(90.0);
	turnToAngle(0, halfRobotLengthIn * 1.0);
	setIntakeHookMode(1);
	driveAndTurnDistanceTiles(0.5, 0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Put mobile goal in corner
	turnToAngle(-100.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-0.7, -100.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Go to neutral wall stake & store 1 ring
	setIntakeHookMode(0);
	setIntakeBottomState(1);
	setIntakeTopState(0);
	turnToAngle(-95);
	driveAndTurnDistanceTiles(2.46, -95, 80.0, 100.0, defaultMoveTilesErrorRange, 2.0);

	// Score ring at neutral wall stake
	turnToAngle(0);
	setArmHangState(1);
	task::sleep(400);
	driveAndTurnDistanceTiles(1.0, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setArmHangState(0);
	driveAndTurnDistanceTiles(0.1, 0.0, 50.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	task::sleep(200);

	/* Red up */

	// Store 1 more ring
	driveAndTurnDistanceTiles(-0.6, 0.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(-135.5);
	driveAndTurnDistanceTiles(1.43, -135.5, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Grab top mobile goal
	turnToAngle(-270.0);
	driveAndTurnDistanceTiles(-1.0, -270.0, 40.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	setGoalClampState(1);

	// Score 2 stored rings
	setIntakeState(1);
	task::sleep(600);

	// Take in 4 rings & score
	setRotation(90.0);
	turnToAngle(0);
	driveAndTurnDistanceTiles(1.51, 0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(118.0);
	driveAndTurnDistanceTiles(1.10, 118.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	turnToAngle(270.0);
	driveAndTurnDistanceTiles(1.50, 270.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Place mobile goal in corner
	turnToAngle(170.0);
	setGoalClampState(0);
	driveAndTurnDistanceTiles(-0.70, 170.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Climb
	setIntakeState(0);
	turnToAngle(135.0);
	setArmHangState(1);
	driveAndTurnDistanceTiles(2.35, 135.0, 60.0, 100.0, defaultMoveTilesErrorRange, 2.0);
	driveAndTurnDistanceTiles(1.0, 135.0, 30.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-0.5, 135.0, 20.0, 100.0, defaultMoveTilesErrorRange, 1.5);
}
