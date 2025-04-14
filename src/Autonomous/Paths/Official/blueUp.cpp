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
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.35_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.25);
	setArmStage(7);
	waitUntil(local::_isDriveAndTurnSettled);

	// (2, 4) goal
	runFollowSpline(robotChassis, "bu grab 1", false);
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	// Top ring
	setIntakeState(1);
	runFollowSpline(robotChassis, "bu ring 1-1a");
	waitUntil(follow::_isPathFollowCompleted);
	runFollowSpline(robotChassis, "bu ring 1-1b", false);
	waitUntil(follow::_isPathFollowCompleted);
	// (2, 5) ring
	global::turnToFace(robotChassis, global::turnToFace_params(6_tiles - 2_tiles, 5_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 2_tiles, 5_tiles), false);
	// Corner rings
	runFollowSpline("bu ring 2-1");
	waitUntil(follow::_isPathFollowCompleted);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-5_in, robotChassis.getLookPose().getRotation()), false);
	setIntakeLiftState(1);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(10_in, robotChassis.getLookPose().getRotation()), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1_tiles, 5_tiles, true), false);
	runFollowSpline(robotChassis, "bu ring 3-1");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 1.0);
	setArmStage(5);
	waitUntil(follow::_isPathFollowCompleted);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(6_tiles - 2_tiles, 3_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1.5_tiles, 3_tiles), true);
	waitUntil(global::_linearPathDistanceError < 0.2);
	setIntakeState(0);
	waitUntil(global::_isDriveToPointSettled);
}

}
