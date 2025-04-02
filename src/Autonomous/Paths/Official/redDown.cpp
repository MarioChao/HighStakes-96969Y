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
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(130.5_qtIn, 90-112.25), true);

	// Rush goal
	waitUntil(local::_driveDistanceError_tiles < (2.0_in).tiles());
	setIntakeState(0, 0.15);
	setSwingState(0);
	waitUntil(local::_isDriveAndTurnSettled);

	// Go back & un-deploy
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-80_qtIn, 90-112.25), true);
	waitUntil(local::_driveDistanceError_tiles < (5.0_in).tiles());
	setSwingState(1);
	waitUntil(local::_isDriveAndTurnSettled);

	// Grab 4th goal
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-220), false);
	setSwingState(0);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-56_qtIn, 90-220, 40.0), true);
	waitUntil(local::_driveDistanceError_tiles < (1.0_in).tiles());
	setGoalClampState(1);

	// Score on goal
	setIntakeState(1);
	wait(700, msec);
	waitUntil(local::_isDriveAndTurnSettled);
	setGoalClampState(0);
	setIntakeState(0);

	// Grab rushed goal
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(46_qtIn, 90-220), false);
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-290), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-66_qtIn, 90-290, 40.0), true);
	waitUntil(local::_driveDistanceError_tiles < (1.0_in).tiles());
	setGoalClampState(1);
	waitUntil(local::_isDriveAndTurnSettled);

	// Take in preload and score
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(118_qtIn, 90-290), false);
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-255), false);
	setIntakeState(1);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(52_qtIn, 90-255), false);

	// Take in corner ring to lady brown
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-192), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(110_qtIn, 90-192, 80.0), true);
	waitUntil(local::_driveDistanceError_tiles < (5.0_in).tiles());
	setArmStage(1);
	waitUntil(local::_isDriveAndTurnSettled);
	wait(200, msec);

	// Release goal
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-40_qtIn, 90-192), false);
	setGoalClampState(0);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(20_qtIn, 90-192), false);

	// Prepare to score wall stake
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-270), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-171_qtIn, 90-270), false);
	setIntakeState(0);

	// Score on wall stake
	local::turnToAngle(robotChassis, local::turnToAngle_params(90-130), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(33_qtIn, 90-130, 60.0), true);
	waitUntil(local::_driveDistanceError_tiles < (2.0_in).tiles());
	setArmStage(4);
	waitUntil(local::_isDriveAndTurnSettled);

	return;
}
}
