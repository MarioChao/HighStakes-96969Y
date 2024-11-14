#include "Autonomous/autonPaths.h"

/// @brief Run the 15-seconds red-down autonomous.
void autonpaths::runAutonRedDown() {
	timer autontimer;
	setRotation(-75.0);

	// Grab middle goal
	driveAndTurnDistanceTiles(-2.35, -75.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.5);
	setGoalClampState(1, 0);
	task::sleep(200);

	// Score preload
	setIntakeState(1);
	driveAndTurnDistanceTiles(0.4, -75.0, 60.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	task::sleep(700);
}
