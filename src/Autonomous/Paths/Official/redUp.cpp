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
	// Alliance wall stake
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.24_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.21);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);
	wait(100, msec);

	// (2, 4) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 4_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 4_tiles, 0_tiles, true, 30), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// Top ring
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 5_tiles, 0.5_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.7_tiles, 6_tiles, 0.4_tiles, false, 60, 1.5), true);
	waitUntil(global::_driveToPointDistanceError < 0.1);
	// Back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.35_tiles, 4.1_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.8_tiles, 4.8_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.6);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 5.5_tiles, 0.3_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Corner ring 1
	setArmStage(6);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.7_tiles, 5.3_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0_tiles, 6_tiles, 0, false, 40, 1.0), false);
	wait(0.5, sec);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.9_tiles, 5_tiles, 0, true), true);
	setIntakeStoreRing(true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.9_tiles, 3_tiles, 0, false, 60), true);
	setArmStage(3);
	waitUntil(global::_driveToPointAngleError_degrees < 60.0);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointAngleError_degrees < 45.0);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 1.5);
	setIntakeLiftState(true);
	waitUntil(global::_driveToPointDistanceError < 1.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 3_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setIntakeLiftState(false);
	// Back up
	wait(0.5, sec);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.8_tiles, 4.2_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(3_tiles, 3_tiles), true);
	waitUntil(global::_turnToFaceError_degrees < 20);
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 1.3_tiles, false, 70, 1.0), true);
	waitUntil(_autonTimer.time(sec) > 14.5);
	setArmStage(20, 0, 40);
	waitUntil(global::_isDriveToPointSettled);
}

}
