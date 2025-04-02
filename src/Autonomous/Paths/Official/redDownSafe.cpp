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
	setRobotPosition(1.0, 1.5);
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
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(140.5_qtIn, 90-112.25), true);

	// Rush goal
	waitUntil(local::_driveDistanceError_tiles < (2.0_in).tiles());
	setIntakeState(0, 0.25);
	setSwingState(0);
	waitUntil(local::_isDriveAndTurnSettled);

	// Go back & un-deploy
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-94_qtIn, 90-112.25), true);
	waitUntil(local::_driveDistanceError_tiles < (9.0_in).tiles());
	setSwingState(1);
	waitUntil(local::_isDriveAndTurnSettled);

	// Grab 4th goal
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-220), false);
	setSwingState(0);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-70_qtIn, 90-220, { {0, 100}, {(40_qtIn).tiles(), 20.0} }), true);
	waitUntil(local::_driveDistanceError_tiles < (2.0_in).tiles());
	setGoalClampState(1);

	// Score on goal
	setIntakeState(1);
	wait(700, msec);
	waitUntil(local::_isDriveAndTurnSettled);
	setGoalClampState(0);
	setIntakeState(-1);
	setIntakeState(0, 0.5);

	// Grab rushed goal
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(66_qtIn, 90-220), false);
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-288), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-82_qtIn, 90-288, { {0, 100}, {(50_qtIn).tiles(), 20.0} }), true);
	waitUntil(local::_driveDistanceError_tiles < (2.0_in).tiles());
	setGoalClampState(1);
	waitUntil(local::_isDriveAndTurnSettled);

	// Take in preload and score
	setIntakeState(1);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(126_qtIn, 90-288), false);
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-255), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(68_qtIn, 90-255), false);

	// Take in corner ring(s) and score
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-200), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(130_qtIn, 90-200, { {0, 100}, {(80_qtIn).tiles(), 30.0} }), false);
	wait(400, msec);

	// Take in again (not implemented)

	// Go near middle
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-260), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-110_qtIn, 90-260), false);

	return;
}
}
