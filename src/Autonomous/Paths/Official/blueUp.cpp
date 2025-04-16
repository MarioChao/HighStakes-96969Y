#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the 15-seconds new blue-up autonomous.
void autonpaths::runAutonBlueUp() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(6-(0.86), 3.46);
	setRobotRotation(121.5);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	doAuton();
}

namespace {

void doAuton() {
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.25_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.22);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);

	// (2, 4) goal
	runFollowSpline(robotChassis, "bu grab 1", false);
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	// Top ring
	setIntakeState(1);
	runFollowSpline(robotChassis, "bu ring 1-1a");
	waitUntil(follow::_isPathFollowCompleted);
	runFollowSpline(robotChassis, "bu ring 1-1b", false);
	waitUntil(follow::_isPathFollowCompleted);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 2_tiles, 5_tiles), true);
	waitUntil(global::_linearPathDistanceError < 0.4);
	// Corner ring 1
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1_tiles, 5_tiles), true);
	waitUntil(global::_linearPathDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 0_tiles, 6_tiles, false, 50, 1.0), false);
	// Corner ring 2
	// local::driveAndTurn(robotChassis, local::driveAndTurn_params(-5_in, robotChassis.getLookRotation()), false);
	// setIntakeLiftState(1);
	// local::driveAndTurn(robotChassis, local::driveAndTurn_params(10_in, robotChassis.getLookRotation(), 50), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1_tiles, 5_tiles, true), true);
	waitUntil(global::_linearPathDistanceError < 0.4);
	setIntakeLiftState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1_tiles, 3.2_tiles), false);
	setIntakeLiftState(0);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(3_tiles, 3_tiles), false);
	setArmStage(5);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(1_tiles, robotChassis.getLookRotation(), 40), true);
	waitUntil(local::_driveDistanceError_tiles < 0.4);
	setIntakeState(0);
	waitUntil(local::_isDriveAndTurnSettled);
}

}
