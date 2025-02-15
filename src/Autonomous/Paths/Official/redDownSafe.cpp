#include "Autonomous/autonPaths.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;
	using namespace autonpaths::combination;

	void loadPaths(int section);

	void doAuton();
}

/// @brief Run the 15-seconds new red-down autonomous.
void autonpaths::runAutonRedDownSafe() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set position and rotation
	mainOdometry.printDebug();
	// mainOdometry.setPosition(0.56, 0.45);
	setRobotRotation(112.25);
	mainOdometry.printDebug();

	// Set config
	// setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	// waitUntil(isArmResetted());


	/* Auton */
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
		async_driveAndTurnDistance_qtInches(132.5, 112.25);
		
		// Rush goal
		waitUntil(_driveDistanceError_inches < 2.0);
		setIntakeState(0, 0.2);
		setSwingState(0);
		waitUntil(_isDriveAndTurnSettled);

		// Go back & un-deploy
		async_driveAndTurnDistance_qtInches(-86, 112.25);
		waitUntil(_driveDistanceError_inches < 9.0);
		setSwingState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Grab 4th goal
		turnToAngle(220);
		setSwingState(0);
		async_driveAndTurnDistance_qtInches(-70, 220, 20.0);
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
		async_driveAndTurnDistance_qtInches(66, 220);
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(293);
		async_driveAndTurnDistance_qtInches(-82, 293, 20.0);
		waitUntil(_driveDistanceError_inches < 2.0);
		setGoalClampState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Take in preload and score
		setIntakeState(1);
		async_driveAndTurnDistance_qtInches(126, 293);
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(255);
		async_driveAndTurnDistance_qtInches(68, 255);
		waitUntil(_isDriveAndTurnSettled);

		// Take in corner ring(s) and score
		turnToAngle(200);
		async_driveAndTurnDistance_qtInches(130, 200, 20.0);
		waitUntil(_isDriveAndTurnSettled);
		wait(400, msec);

		// Take in again
		// double deg = mainOdometry.getLookFieldAngle_degrees();
		// async_driveAndTurnDistance_qtInches(-40, deg);
		// waitUntil(_isDriveAndTurnSettled);
		// setIntakeLiftState(1);
		// async_driveAndTurnDistance_qtInches(48, deg, 40.0);
		// waitUntil(_driveDistanceError_inches < 2.0);
		// setIntakeLiftState(0);
		// waitUntil(_isDriveAndTurnSettled);
		// wait(200, msec);

		// Go near middle
		turnToAngle(260);
		async_driveAndTurnDistance_qtInches(-110, 260);
		waitUntil(_isDriveAndTurnSettled);

		return;
	}
}
