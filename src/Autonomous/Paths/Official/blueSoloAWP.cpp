#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the blue solo AWP.
void autonpaths::runBlueSoloAWP() {
	/* Pre auton */

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
	// ToDo: replace with red solo mirrored

	// Alliance wall stake
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.25_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.22);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);

	// (2, 4) goal
	runFollowSpline(robotChassis, "bsa grab 1", false);
	waitUntil(follow::_ramseteFollowDistanceRemaining_tiles < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(follow::_ramseteFollowDistanceRemaining_tiles < 0.15);
	// Top ring
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 2.6_tiles, 4.6_tiles), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.5_tiles, robotChassis.getLookRotation()), false);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1.9_tiles, 5.1_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 1_tiles, 4_tiles), false);
	runFollowSpline("bsa ring mid");
	waitUntil(follow::_ramseteFollowDistanceRemaining_tiles < 1.5);
	setIntakeFilterEnabled(false);
	setIntakeStoreRing(true);
	setGoalClampState(false);
	waitUntil(follow::_isRamsetePathFollowCompleted);
	// (2, 2) goal
	global::turnToFace(robotChassis, global::turnToFace_params(6_tiles - 2_tiles, 2_tiles, true), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(
		-1_tiles, 0_polarDeg,
		{ {0, 100}, {0.8, 30} }
	), true);
	waitUntil(local::_driveDistanceError_tiles < 0.15);
	setGoalClampState(true);
	setIntakeFilterEnabled(true);
	setIntakeStoreRing(false);
	setArmStage(5);
	waitUntil(local::_isDriveAndTurnSettled);
	// (2, 1) ring
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - 2_tiles, 1_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Ladder
	runFollowSpline(robotChassis, "bsa ladder");
	waitUntil(follow::_isRamsetePathFollowCompleted);

	// waitUntil(_autonTimer.time(sec) > 15);
	// setIntakeState(0);
}

}
