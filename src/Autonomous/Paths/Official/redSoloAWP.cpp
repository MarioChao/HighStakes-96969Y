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
	// Top rings
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.82_tiles, 4.86_tiles, 0.4_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.72_tiles, 5.8_tiles, 0.4_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (2, 5) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(2.3_tiles, 4.2_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.9_tiles, 5.1_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Middle ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 4_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 2.7_tiles, 0_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.5);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 2.2_tiles, 0_tiles, false, 15), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	setIntakeFilterEnabled(false);
	setIntakeStoreRing(true);
	setGoalClampState(false);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 1.8_tiles, 0_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (2, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 2_tiles, 0_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	setArmStage(5);
	setGoalClampState(true);
	setIntakeFilterEnabled(true);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// (2, 1) ring
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.9_tiles, 0.8_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.9);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Ladder
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 2.5_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.7);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 0.5_tiles, false, 50.0), false);
}

}
