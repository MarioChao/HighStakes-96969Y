#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the 15-seconds new red-down autonomous.
void autonpaths::runAutonRedDown() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	mainOdometry.setPosition(0.56, 0.45);
	setRobotRotation(112.25);
	mainOdometry.printDebug();

	// Set config
	// setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	// waitUntil(isArmResetted());


	/* Auton */
	// loadPaths(1);
	doAuton();
}

namespace {
	void loadPaths(int section) {
		// Clear
		clearLinear();
		clearSplines();

		if (section == 1) {
		}
	}

	void doAuton() {
		// Store ring + rush goal
		setArmResetDefaultStage(0);
		setIntakeState(1);
		setSwingState(1);
		async_driveAndTurnDistance_qtInches(130.5, 112.25);
		
		// Rush goal
		waitUntil(_driveDistanceError_inches < 2.0);
		setIntakeState(0, 0.15);
		setSwingState(0);
		waitUntil(_isDriveAndTurnSettled);

		// Go back & un-deploy
		async_driveAndTurnDistance_qtInches(-80, 112.25);
		waitUntil(_driveDistanceError_inches < 5.0);
		setSwingState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Grab 4th goal
		turnToAngle(220);
		setSwingState(0);
		async_driveAndTurnDistance_qtInches(-56, 220, 40.0);
		waitUntil(_driveDistanceError_inches < 1.0);
		setGoalClampState(1);
		
		// Score on goal
		setIntakeState(1);
		wait(700, msec);
		waitUntil(_isDriveAndTurnSettled);
		setGoalClampState(0);
		setIntakeState(0);

		// Grab rushed goal
		async_driveAndTurnDistance_qtInches(46, 220);
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(290);
		async_driveAndTurnDistance_qtInches(-66, 290, 40.0);
		waitUntil(_driveDistanceError_inches < 1.0);
		setGoalClampState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Take in preload and score
		async_driveAndTurnDistance_qtInches(118, 290);
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(255);
		setIntakeState(1);
		async_driveAndTurnDistance_qtInches(52, 255);
		waitUntil(_isDriveAndTurnSettled);

		// Take in corner ring to lady brown
		turnToAngle(192);
		async_driveAndTurnDistance_qtInches(110, 192, 80.0);
		waitUntil(_driveDistanceError_inches < 5.0);
		setArmStage(1);
		waitUntil(_isDriveAndTurnSettled);
		wait(200, msec);

		// Release goal
		async_driveAndTurnDistance_qtInches(-40, 192);
		waitUntil(_isDriveAndTurnSettled);
		setGoalClampState(0);
		async_driveAndTurnDistance_qtInches(20, 192);
		waitUntil(_isDriveAndTurnSettled);

		// Prepare to score wall stake
		turnToAngle(270);
		async_driveAndTurnDistance_qtInches(-171, 270);
		waitUntil(_isDriveAndTurnSettled);
		setIntakeState(0);

		// Score on wall stake
		turnToAngle(130);
		async_driveAndTurnDistance_qtInches(33, 130, 60.0);
		waitUntil(_driveDistanceError_inches < 2.0);
		setArmStage(4);
		waitUntil(_isDriveAndTurnSettled);

		return;
	}
}
