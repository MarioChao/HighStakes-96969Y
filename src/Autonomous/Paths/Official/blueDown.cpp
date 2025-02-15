#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the 15-seconds new blue-down autonomous.
void autonpaths::runAutonBlueDown() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	// mainOdometry.setPosition(5.43, 1.53);
	setRobotRotation(-67.75);
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
		async_driveAndTurnDistance_qtInches(128.5, -(67.75));
		
		// Rush goal
		waitUntil(_driveDistanceError_inches < 2.0);
		setIntakeState(0, 0.2);
		setSwingState(0);
		waitUntil(_isDriveAndTurnSettled);

		// Go back & un-deploy
		async_driveAndTurnDistance_qtInches(-82, -(67.75));
		waitUntil(_driveDistanceError_inches < 5.0);
		setSwingState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Grab 4th goal
		turnToAngle(-(220));
		setSwingState(0);
		async_driveAndTurnDistance_qtInches(-64, -(220), 30.0);
		waitUntil(_driveDistanceError_inches < 2.0);
		setGoalClampState(1);
		
		// Score on goal
		setIntakeState(1);
		wait(700, msec);
		waitUntil(_isDriveAndTurnSettled);
		setGoalClampState(0);
		setIntakeState(-1);
		setIntakeState(0, 0.5);

		// Grab rushed goal
		async_driveAndTurnDistance_qtInches(60, -(220));
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(-(293));
		async_driveAndTurnDistance_qtInches(-74, -(293), 30.0);
		waitUntil(_driveDistanceError_inches < 2.0);
		setGoalClampState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Take in preload and score
		setIntakeState(1);
		async_driveAndTurnDistance_qtInches(118, -(293));
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(-(255));
		async_driveAndTurnDistance_qtInches(68, -(255));
		waitUntil(_isDriveAndTurnSettled);

		// Take in corner ring(s) and score
		turnToAngle(-(192));
		async_driveAndTurnDistance_qtInches(130, -(192), 30.0);
		waitUntil(_isDriveAndTurnSettled);
		wait(400, msec);

		// Go near middle
		turnToAngle(-(230));
		async_driveAndTurnDistance_qtInches(-110, -(230));
		waitUntil(_isDriveAndTurnSettled);

		return;
	}
}
