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
		async_driveAndTurnDistance_qtInches(140.5, -(67.75));
		
		// Rush goal
		waitUntil(_driveDistanceError_inches < 2.0);
		setIntakeState(0, 0.25);
		setSwingState(0);
		waitUntil(_isDriveAndTurnSettled);

		// Go back & un-deploy
		async_driveAndTurnDistance_qtInches(-94, -(67.75));
		waitUntil(_driveDistanceError_inches < 9.0);
		setSwingState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Grab 4th goal
		turnToAngle(-(200));
		setSwingState(0);
		async_driveAndTurnDistance_qtInches(-110, -(200), {{0, 100.0}, {60, 20.0}});
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
		async_driveAndTurnDistance_qtInches(114, -(200));
		waitUntil(_isDriveAndTurnSettled);
		turnToAngle(-(270));
		async_driveAndTurnDistance_qtInches(-70, -(270), 20.0);
		waitUntil(_driveDistanceError_inches < 2.0);
		setGoalClampState(1);
		waitUntil(_isDriveAndTurnSettled);

		// Take in preload and score
		setIntakeState(1);
		async_driveAndTurnDistance_qtInches(170, -(270), {{0, 100}, {140, 30}});
		waitUntil(_isDriveAndTurnSettled);

		// Take in corner ring(s) and score
		turnToAngle(-(200));
		async_driveAndTurnDistance_qtInches(130, -(200), 30.0);
		waitUntil(_isDriveAndTurnSettled);
		wait(400, msec);

		// Go near middle
		turnToAngle(-(260));
		async_driveAndTurnDistance_qtInches(-110, -(260));
		waitUntil(_isDriveAndTurnSettled);

		return;
	}
}
