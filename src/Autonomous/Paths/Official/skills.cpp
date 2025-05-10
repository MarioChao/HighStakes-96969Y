#include "Autonomous/autonPaths.h"

namespace {
using namespace autonpaths;
using namespace autonpaths::pathbuild;
using namespace autonpaths::combination;

void doAuton();
}

/// @brief Run the autonomous skills.
void autonpaths::runAutonSkills() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Set config
	setRobotPosition(1, 3);
	setRobotRotation(-90.0);
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());


	/* Skills */
	doAuton();
}

namespace {
void doAuton() {
	// Alliance wall stake
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(0.27_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.21);
	setArmStage(20);
	waitUntil(local::_isDriveAndTurnSettled);
	wait(0.15, sec);


	/* First goal start */
	// Back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 3_tiles, 1_tiles, true), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (1, 2) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 2_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 2_tiles, 0, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// Rings
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 2_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 1.4_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 1_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 0.5_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.3);
	setIntakeStoreRing(true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Back up & hold ring in arm
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 1.4_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.8);
	setArmStage(1);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointDistanceError < 0.6);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Wall stake ring 1
	setArmStage(3);
	setIntakeState(-1);
	setIntakeState(1, 0.3);
	setIntakeStoreRing(true, 0.3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, -3_tiles, 0, false, 40, 1.0), false);
	setArmStage(20);
	wait(0.4, sec);
	// Wall stake ring 2 in arm
	setArmStage(1);
	setIntakeStoreRing(false);
	wait(0.4, sec);
	setIntakeState(1);
	wait(0.2, sec);
	// Wall stake ring 2
	setArmStage(20);
	setIntakeState(-1);
	setIntakeState(0, 0.3);
	wait(0.4, sec);
	// Back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 1_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Rings
	setArmStage(0);
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 1_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 1.1_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.5_tiles, 1.1_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 0.5_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Place mobile goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(-3_tiles, -3_tiles, 0, true, 40), true);
	wait(0.8, sec);
	setGoalClampState(false);
	wait(0.2, sec);
	/* First goal end */


	/* 2nd goal start */
	// (1, 4) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(1.2_tiles, 2_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 4_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 4_tiles, 0, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// ---------- same thing but mirrored ----------
	// Rings
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 6_tiles - (2_tiles), 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 6_tiles - (1.4_tiles), 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 6_tiles - (1_tiles), 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 6_tiles - (0.5_tiles), 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.3);
	setIntakeStoreRing(true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Back up & hold ring in arm
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 6_tiles - (1.4_tiles), 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.8);
	setArmStage(1);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointDistanceError < 0.6);
	setIntakeState(1);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Wall stake ring 1
	setArmStage(3);
	setIntakeState(-1);
	setIntakeState(1, 0.3);
	setIntakeStoreRing(true, 0.3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 6_tiles - (-3_tiles), 0, false, 40, 1.0), false);
	setArmStage(20);
	wait(0.4, sec);
	// Wall stake ring 2 in arm
	setArmStage(1);
	setIntakeStoreRing(false);
	wait(0.4, sec);
	setIntakeState(1);
	wait(0.2, sec);
	// Wall stake ring 2
	setArmStage(20);
	setIntakeState(-1);
	setIntakeState(0, 0.3);
	wait(0.4, sec);
	// Back up
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 6_tiles - (1_tiles), 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Rings
	setArmStage(0);
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(2_tiles, 6_tiles - (1_tiles), 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 6_tiles - (1.1_tiles), 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(0.5_tiles, 6_tiles - (1.1_tiles), 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(1_tiles, 6_tiles - (0.5_tiles), 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Place mobile goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(-3_tiles, 6_tiles - (-3_tiles), 0, true, 40), true);
	wait(0.8, sec);
	setGoalClampState(false);
	wait(0.2, sec);
	/* 2nd goal end */


	/* 3rd goal start */
	// (5, 5) ring
	setIntakeState(0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.6_tiles, 4.6_tiles), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	setIntakeStoreRing(true);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 5_tiles, 0.2_tiles, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// (5, 3) goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.8_tiles, 4_tiles, 0_tiles, true, 80), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 3_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 3_tiles, 0, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	setIntakeStoreRing(false);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// Rings
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 4_tiles, 0_tiles, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 0_tiles, false, 30), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4_tiles, 2_tiles, 0_tiles, false, 30), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 1_tiles, 0_tiles, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 1_tiles, 0.2_tiles, false, 30), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Place mobile goal
	wait(0.5, sec);
	global::driveToPoint(robotChassis, global::driveToPoint_params(9_tiles, -3_tiles, 0, true, 30), true);
	wait(0.8, sec);
	setGoalClampState(false);
	wait(0.2, sec);
	/* 3rd goal end */
	
	
	/* 4th goal start */
	// (5.5, 4) goal
	setIntakeState(0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.5_tiles, 2_tiles, 0, false, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 4_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 1.0);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 4_tiles, 0, true, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.25);
	setArmStage(0);
	setGoalClampState(true);
	waitUntil(global::_driveToPointDistanceError < 0.15);
	// (5.5, 5) ring to arm
	setArmStage(1);
	setIntakeState(1);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 5_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	wait(0.5, sec);
	// (5.5, 5) ring
	setArmStage(3);
	setIntakeState(-1);
	setIntakeState(1, 0.3);
	setIntakeStoreRing(true, 0.3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5.5_tiles, 5.5_tiles, 0, false, 40), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Place mobile goal
	global::driveToPoint(robotChassis, global::driveToPoint_params(9_tiles, 9_tiles, 0, true, 30), true);
	wait(0.8, sec);
	setGoalClampState(false);
	wait(0.2, sec);	
	/* 4th goal end */
	
	
	/* Blue alliance stake start */
	// Go to wall stake
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.6_tiles, 3_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.2);
	// Wall stake ring 1
	global::driveToPoint(robotChassis, global::driveToPoint_params(9_tiles, 3_tiles, 0, false, 40, 1.0), false);
	local::driveAndTurn(robotChassis, local::driveAndTurn_params(-0.15_tiles, robotChassis.getLookRotation()), true);
	waitUntil(local::_driveDistanceError_tiles < 0.1);
	setArmStage(20);
	wait(0.4, sec);
	// Wall stake ring 2 in arm
	setArmStage(1);
	setIntakeStoreRing(false);
	wait(0.4, sec);
	setIntakeState(1);
	wait(0.2, sec);
	// Wall stake ring 2
	setArmStage(20);
	setIntakeState(-1);
	setIntakeState(0, 0.3);
	wait(0.4, sec);
	/* Blue alliance stake end */


	/* Climb start */
	// Back up
	setArmStage(3);
	global::driveToPoint(robotChassis, global::driveToPoint_params(5_tiles, 3_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Navigate
	global::driveToPoint(robotChassis, global::driveToPoint_params(4.4_tiles, 4_tiles, 0, true, 60), true);
	waitUntil(global::_driveToPointAngleError_degrees < 10);
	setArmStage(9);
	waitUntil(global::_driveToPointDistanceError < 0.4);
	// Climb
	global::driveToPoint(robotChassis, global::driveToPoint_params(3_tiles, 3_tiles, 0, true, 60, 1.0), false);
	setArmStage(0);
	/* Climb end */
}
}
