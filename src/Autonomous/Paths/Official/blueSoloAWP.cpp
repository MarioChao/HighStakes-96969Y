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
	// Mirrored & modified from red solo

	// 4/30
	// Partially from up

	/* Up start */
	// Alliance wall stake
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.24_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.21);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);
	wait(0.15, sec);

	// (2, 4) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2_tiles), 4_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2_tiles), 4_tiles, 0_tiles, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// Top ring
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (3_tiles), 4.8_tiles, 0.35_tiles, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2.71_tiles), 6_tiles, 0.35_tiles, false, 60, 1.2), true);
	waitUntil(global::_driveToPointDistanceError < 0.1);
	wait(0.2, sec);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2.35_tiles), 4.1_tiles, 0_tiles, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	/* Up end */

	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2_tiles), 5.2_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);

	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 4_tiles, 0.3_tiles, false, 50), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	wait(0.3, sec);
	setGoalClampState(false);
	// Middle ring: slow down & back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 3_tiles, 0_tiles, false, 70), true);
	waitUntil(global::_driveToPointDistanceError < 0.8);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 2.2_tiles, 0_tiles, false, 20), true);
	wait(0.3, sec);
	setArmStage(3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 2.2_tiles, 0_tiles, false, 5), true);
	wait(0.3, sec);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 4_tiles, 0_tiles, true, 40), true);
	wait(0.3, sec);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 2.2_tiles, 0_tiles, false, 30), true);
	setIntakeStoreRing(true);
	wait(1.0, sec);
	// Middle ring: accelerate
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (1_tiles), 1_tiles, 0.25_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (2, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2_tiles), 2_tiles, 0_tiles, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2_tiles), 2_tiles, 0_tiles, true, 40), true);
	setArmStage(5);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setIntakeStoreRing(false);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// (2, 1) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2_tiles), 0.7_tiles, 0.2_tiles), true);
	waitUntil(global::_driveToPointAngleError_degrees < 30.0);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Ladder
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (2.1_tiles), 2.2_tiles), true);
	setIntakeStoreRing(true);
	waitUntil(global::_driveToPointAngleError_degrees < 60.0);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointAngleError_degrees < 45.0);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.5);
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles - (3_tiles), 3_tiles, 1.1_tiles, false, 70, 1.0), true);
	waitUntil(_autonTimer.time(sec) > 14.7);
	setArmStage(20, 0, 40);
	waitUntil(global::_isDriveToPointSettled);
}

}
