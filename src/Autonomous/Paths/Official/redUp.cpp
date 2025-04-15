#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the 15-seconds red-up autonomous.
void autonpaths::runAutonRedUp() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(0.86, 3.46);
	setRobotRotation(-121.5);
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
	setArmStage(7);
	waitUntil(local::_isDriveAndTurnSettled);

	// (2, 4) goal
	runFollowSpline(robotChassis, "ru grab 1", false);
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	// Top ring
	setIntakeState(1);
	runFollowSpline(robotChassis, "ru ring 1-1a");
	waitUntil(follow::_isPathFollowCompleted);
	runFollowSpline(robotChassis, "ru ring 1-1b", false);
	waitUntil(follow::_isPathFollowCompleted);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 5_tiles), true);
	waitUntil(global::_linearPathDistanceError < 0.3);
	// Corner rings
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 5_tiles), true);
	waitUntil(global::_linearPathDistanceError < 0.3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0_tiles, 6_tiles, false, 50), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-5_in, robotChassis.getLookRotation()), false);
	setIntakeLiftState(1);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(10_in, robotChassis.getLookRotation()), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 5_tiles, true), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 3.2_tiles), false);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(2_tiles, 3_tiles), false);
	setArmStage(5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.8_tiles, 3_tiles, false, 50), true);
	waitUntil(global::_linearPathDistanceError < 0.2);
	setIntakeState(0);
	waitUntil(global::_isDriveToPointSettled);
}

}
