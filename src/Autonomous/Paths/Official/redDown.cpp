#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the 15-seconds red-down autonomous.
void autonpaths::runAutonRedDown() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(0.86, 2.54);
	setRobotRotation(-58.5);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	doAuton();
}

namespace {

void doAuton() {
	// 5/1

	// if (true) {
	/* Down start */
	// Alliance wall stake
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.24_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.21);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);
	wait(0.15, sec);

	// (1, 3) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.265_tiles, 2.32_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setArmStage(0);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	local::turnToAngle(robotChassis, local::turnToAngle_params(130), false);
	setSwingState_right(true);
	wait(0.1, sec);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.45_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.2);

	// (2, 2) goal
	global::turnToFace(robotChassis, global::turnToFace_params(2.1_tiles, 2_tiles, true, 40), true);
	waitUntil(global::_turnToFaceError_degrees < 5.0);
	setSwingState_right(false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 2._tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 2_tiles, 0_tiles, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	/* Down end */
	// }


	/* Middle start */
	// Middle ring 1
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.3_tiles, 2.6_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	local::turnToAngle(robotChassis, local::turnToAngle_params(35_polarDeg, 50, 0.1), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 0.45_tiles, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setSwingState_right(1);
	wait(0.2, sec);
	// Middle ring 2 (tight angle)
	local::turnToAngle(robotChassis, local::turnToAngle_params(25_polarDeg, 50, 0.05), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.8_tiles, 3.2_tiles, 1.1_tiles, false, 60, 0.7), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setSwingState_left(1);
	wait(0.2, sec);
	return;
	/* Middle end */


	// Back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 1_tiles, 0.6_tiles, true, 80), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Release rings
	setSwingState_left(0);
	setSwingState_right(0);
	wait(0.2, sec);
	// Take in rings
	setIntakeState(1);
	setArmStage(3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 2_tiles, 0, false, 40), true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.7_tiles, 2.3_tiles, 0, false, 50), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.3_tiles, 1.7_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// (2, 1) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 0.5_tiles, 0.5_tiles, false, 50), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);

	// Corner ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.5_tiles, 0.4_tiles, 0, false, 50), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setArmStage(7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.7_tiles, 0.5_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::turnToFace(robotChassis, global::turnToFace_params(0_tiles, 0_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0_tiles, 0_tiles, -0.5_tiles, false, 30, 1.0), false);
	wait(0.3, sec);
	// Corner ring 2nd time
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 1_tiles, 0.4_tiles, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0_tiles, 0_tiles, 0.4_tiles, false, 60, 0.5), false);

	// Drop goal near corner
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 1_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setArmStage(5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 2_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointAngleError_degrees < 30);
	setGoalClampState(false);
	waitUntil(global::_driveToPointDistanceError < 0.2);

	// Touch ladder
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 1.2_tiles, false, 70, 1.0), true);
	waitUntil(_autonTimer.time(sec) > 14.7);
	setArmStage(20, 0, 40);
	waitUntil(global::_isDriveToPointSettled);
}

}
