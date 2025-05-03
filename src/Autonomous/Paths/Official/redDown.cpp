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
	// 5/3 - 1779X's route https://youtu.be/esmWptONZn0?t=94

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
	local::turnToAngle(robotChassis, local::turnToAngle_params(130_polarDeg), false);
	setSwingState_right(true);
	wait(0.1, sec);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.23_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.13);

	// (2, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 2_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 2_tiles, 0_tiles, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	/* Down end */


	/* Middle start */
	// Re-position swinged ring
	global::turnToFace(robotChassis, global::turnToFace_params(2.5_tiles, 3.05_tiles), false);
	wait(0.2, sec);
	setSwingState_right(false);
	wait(0.1, sec);
	setIntakeStoreRing(true);
	// Middle ring 1
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.05_tiles, 3.05_tiles, 0.5_tiles, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// setIntakeStoreRing(false);
	// setIntakeState(0, 0.1);
	setSwingState_right(true);
	wait(0.1, sec);
	/* Middle end */


	// Back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.7_tiles, 2.2_tiles, 0.1_tiles, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Re-position swinged ring
	// -> opposite curve route to avoid wrong ring
	setArmStage(3);
	local::turnToAngle(robotChassis, local::turnToAngle_params(-55_polarDeg), false);
	waitUntil(local::_turnAngleError_degrees < 10);
	setIntakeStoreRing(false);
	setIntakeState(1, 0.1);
	waitUntil(local::_isTurnToAngleSettled);
	wait(0.1, sec);
	setSwingState_right(false);
	wait(0.1, sec);
	// Take in rings
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.8_tiles, 0.5_tiles, 0.7_tiles, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// (2, 1) ring
	setIntakeStoreRing(true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.4_tiles, 0.8_tiles, 0.2_tiles, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);


	/* Corner start */
	// Navigate to corner
	global::turnToFace(robotChassis, global::turnToFace_params(1.5_tiles, 0.4_tiles), true);
	waitUntil(global::_turnToFaceError_degrees < 10);
	setIntakeStoreRing(false);
	setIntakeState(1, 0.1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.5_tiles, 0.4_tiles, 0, false, 80), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setArmStage(7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.7_tiles, 0.6_tiles, 0.2_tiles, false, 80), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Corner ring 1
	setIntakeAntiJam(false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(-2_tiles, -2_tiles, 2.828_tiles, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.8);
	global::driveToPoint(robotChassis, global::driveToPoint_params(-2_tiles, -2_tiles, 0, false, 40, 0.5), false);
	wait(0.2, sec);
	// Corner ring 2nd time
	// local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.2_tiles, robotChassis.getLookRotation(), {{0, 50}}), false);
	// setIntakeLiftState(true);
	// wait(0.1, sec);
	// local::driveAndTurn(robotChassis, local::driveAndTurn_params(1_tiles, robotChassis.getLookRotation(), {{0, 30}}, 100, 0.5), false);
	// setIntakeLiftState(false);
	// wait(0.3, sec);
	/* Corner end */


	// Corner ring double check
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.3_tiles, robotChassis.getLookRotation(), 20), true);
	wait(0.5, sec);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.1_tiles, robotChassis.getLookRotation(), 5), true);
	wait(1.2, sec);
	// Drop goal near corner
	setArmStage(5);
	setIntakeAntiJam(true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 1.9_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointAngleError_degrees < 30);
	setGoalClampState(false);
	setIntakeState(0, 0.5);
	waitUntil(global::_driveToPointDistanceError < 0.7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.1_tiles, 1.9_tiles, 0, false, 100), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);

	// Touch ladder
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.1_tiles, 2.9_tiles, 1.2_tiles, false, 70, 1.0), true);
	waitUntil(_autonTimer.time(sec) > 14.7);
	setArmStage(20, 0, 40);
	waitUntil(global::_isDriveToPointSettled);
}

}
