#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;

void doAuton();
}


void autonpaths::runAutonBlueDownLBRush() {
	/* Pre auton */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(5.18, 0.52);
	setRobotRotation(-90.0);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Auton */
	doAuton();
}


namespace {

void doAuton() {
	// Goal rush
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.5_tiles, 0.52_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.9);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 1_tiles, 0.5_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setArmStage(6);
	waitUntil(global::_driveToPointDistanceError < 0.1);
	// Move goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 0.4_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setArmStage(20);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	local::turnToAngle(robotChassis, local::turnToAngle_params(180_polarDeg), false);
	setArmStage(3);

	// (4, 1) ring
	setIntakeState(1);
	setIntakeStoreRing(true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 1.1_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// (4, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 2_tiles, 0.5_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setArmStage(0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 1.3_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.8_tiles, 2.2_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.3);
	setGoalClampState(true);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Corner ring 1
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles, 0_tiles, 1_tiles), false);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles, 0_tiles, 0, false, 50, 1.0), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 1.5_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 2_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setIntakeLiftState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 2.8_tiles), false);
	setIntakeLiftState(0);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(4_tiles, 3_tiles), false);
	setArmStage(5);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(1_tiles, robotChassis.getLookRotation(), 40), false);

	// waitUntil(_autonTimer.time(sec) > 15);
	// setIntakeState(0);
}

}
