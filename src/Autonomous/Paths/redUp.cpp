#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds new red-up autonomous.
void autonpaths::runAutonRedUp() {
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
	driveAndTurnDistanceTiles(-1.73, -115.0, 38.0, 15.0, defaultMoveTilesErrorRange, 2.0);
	// turnToAngle(-80.0, -halfRobotLengthIn * 1.0);
	task::sleep(200);

	// Intake middle up
	turnToAngle(47.0);
	setIntakeState(1);
	driveAndTurnDistanceTiles(0.85, 47.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(-0.12, 47.0, 50.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake 2nd middle up
	turnToAngle(5.0);
	turnToAngleVelocity(50.0, 80.0, halfRobotLengthIn * 1.3);
	task::sleep(200);

	// Intake left up
	turnToAngleVelocity(-88.0, 50.0);
	driveAndTurnDistanceTiles(1.3, -90.0, 70.0, 100.0, defaultMoveTilesErrorRange, 1.5);

	// Intake corner
	setArmHangState(1);
	turnToAngle(-55.0);
	driveAndTurnDistanceTiles(1.65, -55.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	driveAndTurnDistanceTiles(0.7, -45.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.3);
	task::sleep(600);
	driveAndTurnDistanceTiles(-0.50, -45.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);

	// Touch ladder
	turnToAngle(155.0);
	while (autontimer.value() < 13.0) {
		task::sleep(20);
	}
	setArmHangState(0);
	setIntakeState(0, 0.6);
	driveAndTurnDistanceTiles(2.2, 155.0, 80.0, 100.0, defaultMoveTilesErrorRange, 1.5);
	driveAndTurnDistanceTiles(0.5, 155.0, 50.0, 100.0, defaultMoveTilesErrorRange, 1.0);
}
