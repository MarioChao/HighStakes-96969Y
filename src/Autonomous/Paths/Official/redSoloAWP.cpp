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
	// Partially from up

	/* Up start */
	// Alliance wall stake
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.24_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.21);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);
	wait(100, msec);

	// (2, 4) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 4_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 4_tiles, 0_tiles, true, 30), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// Top ring
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 5_tiles, 0.45_tiles, false, 30), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.7_tiles, 6_tiles, 0.4_tiles, false, 60, 1.5), true);
	waitUntil(global::_driveToPointDistanceError < 0.1);
	wait(0.2, sec);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.35_tiles, 4.1_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	/* Up end */

	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 5.2_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);

	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 4_tiles, 0_tiles, false, 50), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 3_tiles, 0_tiles, false, 70), true);
	waitUntil(global::_driveToPointAngleError_degrees < 20);
	setIntakeStoreRing(true);
	setGoalClampState(false);
	waitUntil(global::_driveToPointDistanceError < 0.7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 2.2_tiles, 0_tiles, false, 20), true);
	setArmStage(3);
	wait(1.15, sec);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 1.3_tiles, 0.2_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (2, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 2_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 2_tiles, 0_tiles, true, 30), true);
	setArmStage(5);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setGoalClampState(true);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// (2, 1) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 0.7_tiles, 0.2_tiles), true);
	waitUntil(global::_driveToPointAngleError_degrees < 30.0);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Ladder
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 2.2_tiles), true);
	waitUntil(global::_driveToPointAngleError_degrees < 150.0);
	setIntakeStoreRing(true);
	waitUntil(global::_driveToPointAngleError_degrees < 60.0);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointAngleError_degrees < 45.0);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.5);
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 1.2_tiles, false, 70, 1.0), true);
	waitUntil(_autonTimer.time(sec) > 14.5);
	setArmStage(20, 0, 40);
	waitUntil(global::_isDriveToPointSettled);
}

}
