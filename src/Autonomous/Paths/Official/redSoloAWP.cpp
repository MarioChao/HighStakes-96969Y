#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the red solo AWP.
void autonpaths::runRedSoloAWP() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(0.8, 3.7);
	setRobotRotation(-123);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	doAuton();
}

namespace {

void doAuton() {
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.5_tiles, robotChassis.getLookPose().getRotation()), false);
	waitUntil(local::_driveDistanceError_tiles < 0.3);
	setArmStage(4);
	waitUntil(local::_isDriveAndTurnSettled);

	runFollowSpline(robotChassis, "rsa grab 1");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	setIntakeState(true);
	runFollowSpline(robotChassis, "rsa ring 1-1a");
	waitUntil(follow::_isPathFollowCompleted);
	runFollowSpline(robotChassis, "rsa ring 1-1b");
	waitUntil(follow::_isPathFollowCompleted);
	runFollowSpline(robotChassis, "rsa ring 1-2");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 1.2);
	setIntakeLiftState(true);
	setIntakeStoreRing(true);
	waitUntil(follow::_isPathFollowCompleted);
	setGoalClampState(false);
	setIntakeLiftState(false);
	runFollowSpline(robotChassis, "rsa grab 2");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	setIntakeState(true);
	runFollowSpline(robotChassis, "rsa ring 2-1");
	waitUntil(follow::_isPathFollowCompleted);
	runFollowSpline(robotChassis, "rsa ladder");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.5);
	setIntakeState(false);
	setArmStage(4);
	waitUntil(follow::_isPathFollowCompleted);
}
}
