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
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.25_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.22);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);

	// (2, 4) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 4_tiles, 0.1_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// Top ring
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.82_tiles, 4.86_tiles, 0.4_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.7_tiles, 5.8_tiles, 0.4_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.3_tiles, 4.2_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.9_tiles, 5.1_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Corner ring 1
	setArmStage(6);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 5_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0_tiles, 6_tiles, 0, false, 50, 1.0), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 5_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 3_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 1.2);
	setIntakeLiftState(true);
	waitUntil(global::_driveToPointDistanceError < 0.7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 3_tiles, 0, false, 40.0), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setIntakeLiftState(false);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(2_tiles, 3_tiles), false);
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.7);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(1_tiles, robotChassis.getLookRotation(), 30), false);
}

}
