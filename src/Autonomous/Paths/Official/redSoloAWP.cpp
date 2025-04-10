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
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.4_tiles, robotChassis.getLookPose().getRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.25);
	setArmStage(4);
	waitUntil(local::_isDriveAndTurnSettled);

	// (2, 4) goal
	runFollowSpline(robotChassis, "rsa grab 1", false);
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.15);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(follow::_isPathFollowCompleted);
	// Top ring
	setIntakeState(1);
	// runFollowSpline(robotChassis, "rsa ring 1-1a");
	// waitUntil(follow::_isPathFollowCompleted);
	// runFollowSpline(robotChassis, "rsa ring 1-1b", false);
	// waitUntil(follow::_isPathFollowCompleted);
	global::turnToFace(robotChassis, global::turnToFace_params(2.86_tiles, 4.86_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.68_tiles, 4.68_tiles), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.3_tiles, robotChassis.getLookPose().getRotation()), false);
	// (2, 5) ring
	global::turnToFace(robotChassis, global::turnToFace_params(2_tiles, 5_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 5_tiles), false);
	global::turnToFace(robotChassis, global::turnToFace_params(1_tiles, 4_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 4_tiles), false);
	// Middle ring
	setIntakeFilterEnabled(false);
	setIntakeStoreRing(true);
	global::turnToFace(robotChassis, global::turnToFace_params(1_tiles, 2_tiles), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(
		2_tiles, 270_polarDeg, { {0, 100}, {0.7, 20}, {1.4, 100} }
	), true);
	waitUntil(local::_driveDistanceError_tiles < 1.5);
	setGoalClampState(false);
	waitUntil(local::_isDriveAndTurnSettled);
	// (2, 2) goal
	global::turnToFace(robotChassis, global::turnToFace_params(2_tiles, 2_tiles, true), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(
		-1_tiles, 180_polarDeg,
		{ {0, 100}, {0.8, 30} }
	), true);
	waitUntil(local::_driveDistanceError_tiles < 0.15);
	setGoalClampState(true);
	waitUntil(local::_isDriveAndTurnSettled);
	// (2, 1) ring
	setIntakeState(1);
	runFollowSpline(robotChassis, "rsa ring 2-1");
	waitUntil(follow::_isPathFollowCompleted);
	// Ladder
	runFollowSpline(robotChassis, "rsa ladder");
	waitUntil(follow::_pathFollowDistanceRemaining_tiles < 0.5);
	setArmStage(4);
	setIntakeState(0);
	waitUntil(follow::_isPathFollowCompleted);
}

}
