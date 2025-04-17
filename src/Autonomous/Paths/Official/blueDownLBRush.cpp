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
	setArmStage(6);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.2_tiles, 0.52_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPose(robotChassis, global::driveToPose_params(3.5_tiles, 0.7_tiles, 150_polarDeg), true);
	waitUntil(global::_driveToPoseDistanceError.tiles() < 0.2);
	setArmStage(20);
	waitUntil(global::_isDriveToPoseSettled);
	// Move goal
	setArmStage(6);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.4_tiles, robotChassis.getLookRotation()), false);
	setArmStage(0);
	local::turnToAngle(robotChassis, local::turnToAngle_params(220_polarDeg), false);

	// (4, 1) ring
	setIntakeState(1);
	setIntakeStoreRing(true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 1_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// (4, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 1.5_tiles), false);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 2_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	setGoalClampState(true);
	setIntakeStoreRing(false);
	waitUntil(global::_isDriveToPointSettled);
	// Corner ring 1
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 1.5_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.25_tiles, 0.75_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles, 0_tiles, false, 50, 1.0), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 1.5_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 2_tiles, true), false);
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
