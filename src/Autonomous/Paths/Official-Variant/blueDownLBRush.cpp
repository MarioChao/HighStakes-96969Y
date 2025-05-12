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
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.95);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 1_tiles, 0.52_tiles), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.37);
	setArmStage(8);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.1);
	// Move goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.2_tiles, 0.4_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.2);
	setArmStage(9);
	local::turnToAngle(robotChassis, local::turnToAngle_params(190_polarDeg), false);
	setArmStage(3);

	// (4, 1) ring
	setIntakeState(1);
	setIntakeStoreRing(true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 1.3_tiles), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.4);
	// (4, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 2_tiles, 0.8_tiles), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.2);
	setArmStage(0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3.8_tiles, 2.2_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.3);
	setGoalClampState(true);
	setIntakeStoreRing(false);
	setIntakeState(0);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.2);
	// (5, 1) ring remove
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.3_tiles, 1_tiles, 0), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 1_tiles, 0), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.2);
	setArmStage(7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.7_tiles, 1_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.2);
	// Corner ring 1
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles, 0_tiles), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.8);
	global::driveToPoint(robotChassis, global::driveToPoint_params(6_tiles, 0_tiles, 0, false, 50, 0.7), false);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.05_tiles, 1.5_tiles, 0, true), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.05_tiles, 2_tiles, 0.5_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.4);
	setIntakeState(0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 3_tiles), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 1.4);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError.tiles() < 1.2);
	setIntakeLiftState(true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 3_tiles, 0, false, 40.0), true);
	waitUntil(global::_driveToPointDistanceError.tiles() < 0.2);
	setIntakeLiftState(false);
	// Ladder
	global::turnToFace(robotChassis, global::turnToFace_params(4_tiles, 3_tiles), false);
	setArmStage(5);
	waitUntil(_autonTimer.time(sec) > 13.7);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(1_tiles, robotChassis.getLookRotation(), 30), false);

	// waitUntil(_autonTimer.time(sec) > 15);
	// setIntakeState(0);
}

}
