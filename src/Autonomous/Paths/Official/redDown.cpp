#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the 15-seconds new red-down autonomous.
void autonpaths::runAutonRedDown() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(0.9, 1.5);
	setRobotRotation(112.2);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	doAuton();
}

namespace {

void doAuton() {
	setSwingState_left(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.3_tiles, 0.88_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 1_tiles, true), false);
	local::turnToAngle(robotChassis, local::turnToAngle_params(-90_polarDeg), false);
	setSwingState_left(0);

	runFollowSpline("rd grab 2", false);
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	setIntakeState(1);
	runFollowSpline("rd ring 2-1");
	waitUntil(follow::_isPathFollowCompleted);
	local::turnToAngle(robotChassis, local::turnToAngle_params(-90_polarDeg), false);
	setGoalClampState(false);
	setIntakeState(0);
	runFollowSpline("rd grab 3");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	setIntakeState(1);
	runFollowSpline("rd ring 3-1");
	waitUntil(follow::_isPathFollowCompleted);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(10_in, robotChassis.getLookPose().getRotation()), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-10_in, robotChassis.getLookPose().getRotation()), false);
	setIntakeLiftState(1);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(20_in, robotChassis.getLookPose().getRotation()), false);
	setIntakeLiftState(0);
	runFollowSpline("rd ladder 1a");
	waitUntil(follow::_isPathFollowCompleted);
	setIntakeState(0);
	runFollowSpline("rd ladder 1b");
	waitUntil(follow::_isPathFollowCompleted);
}

}
