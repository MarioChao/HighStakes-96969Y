#include "Autonomous/autonPaths.h"

#include "Utilities/generalUtility.h"
#include "Utilities/angleUtility.h"

#include "Simulation/robotSimulator.h"

namespace {
	using namespace autonpaths;
	using namespace autonpaths::pathbuild;

	void firstGoal();
	void secondGoal();
	void thirdGoal();
	void fourthGoal();
	void finalSkills();
}

/// @brief Run the skills autonomous.
void autonpaths::runAutonSkills() {
	/* Pre skills */

	// Timer
	_autonTimer.reset();

	// Pre-process
	loadSkillsSplines(1);

	// Set position and rotation
	mainOdometry.setPosition(0.9, 3);
	mainOdometry.setLookAngle(-90.0);
	mainOdometry.printDebug();
	setRobotRotation(-90.0);
	mainOdometry.restart();
	mainOdometry.printDebug();

	if (mainUseSimulator) {
		robotSimulator.position = Vector3(0.9, 3);
		robotSimulator.angularPosition = genutil::toRadians(angle::swapFieldPolar_degrees(-90));
	}

	// Set config
	setDifferentialUseRelativeRotation(true);

	// Wait for arm reset
	waitUntil(isArmResetted());

	/* Skills */
	firstGoal();

	loadSkillsSplines(2);
	secondGoal();

	loadSkillsSplines(3);
	thirdGoal();

	loadSkillsSplines(4);
	fourthGoal();

	loadSkillsSplines(5);
	finalSkills();
}

namespace {
	void firstGoal() {
		/* Score alliance wall stake and grab goal */

		// Score preload at alliance wall stake
		setArmStage(2);
		task::sleep(600);
		driveAndTurnDistanceTiles(0.5, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		driveAndTurnDistanceTiles(-0.5, -90.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Redirect 1 ring and score more rings */

		// Start intake
		turnToAngle(80.0);
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowSpline();

		// Wait and remove redirect
		waitUntil(_trajectoryPlan.getMotionAtTime(_splinePathTimer.value())[0] >= 1.7);
		printf("No more redirect\n");
		setIntakeToArm(0);

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Score on neutral wall stake */

		// Raise arm
		setArmStage(3);
		setIntakeState(0);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Drive forward and score
		turnToAngle(180.0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(1.5, 180.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, 180.0, 70.0, 100.0, defaultMoveTilesErrorRange, 0.5);


		/* Score more rings */

		// Start intake
		turnToAngle(-80.0);
		setIntakeState(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score 1 ring
		turnToAngle(145);
		driveAndTurnDistanceTiles(1.0, 145, 70.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(60);
		setGoalClampState(0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(-1.0, 60, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, 45, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	}

	void secondGoal() {
		/* Redirect middle ring */

		// Start intake
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Remove redirect
		setIntakeToArm(0);
		setIntakeState(0);


		/* Store 1 ring */

		// Store ring
		setIntakeStoreRing(1);
		turnToAngle(135);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Remove store
		setIntakeStoreRing(0);


		/* Grab mobile goal */

		// Turn
		turnToAngle(-90);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score on alliance wall stake */

		// Turn
		turnToAngle(90);
		setArmStage(2);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on alliance wall stake
		turnToAngle(90.0);
		driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.8);
		driveAndTurnDistanceTiles(-0.5, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);


		/* Score 3 rings */

		// Start intake
		turnToAngle(-30);
		setIntakeState(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score 1 ring
		turnToAngle(145);
		driveAndTurnDistanceTiles(1.0, 145, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Score 2 rings and redirect 1 ring */

		// Turn
		turnToAngle(-135);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Redirect ring
		turnToAngle(40);
		setIntakeToArm(1);
		driveAndTurnDistanceTiles(1.0, 35, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(-25);
		setGoalClampState(0);
		setIntakeState(0);
		driveAndTurnDistanceTiles(-1.0, -25, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, -45, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Score on alliance wall stake */

		// Raise arm
		setArmStage(2);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on alliance wall stake
		turnToAngle(90.0);
		driveAndTurnDistanceTiles(1.5, 90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.8);
		driveAndTurnDistanceTiles(-0.5, 90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);
	}

	void thirdGoal() {
		/* Push mobile goal to corner */

		// Turn
		turnToAngle(180);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Move forward a bit
		driveAndTurnDistanceTiles(0.5, -170, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
	}

	void fourthGoal() {
		/* Redirect 1 ring and store 1 ring */

		// Start intake and redirect
		turnToAngle(-105);
		setIntakeState(1);
		setIntakeToArm(1);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_trajectoryPlan.getMotionAtTime(_splinePathTimer.value())[0] >= 2.0);

		// Remove redirect and start storing
		setIntakeToArm(0);
		setIntakeStoreRing(1);

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Grab mobile goal */

		// Turn
		turnToAngle(90);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Grab goal
		setGoalClampState(1);
		wait(200, msec);


		/* Score on neutral wall stake */

		// Start scoring and raise arm
		setIntakeState(1);
		setArmStage(3);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on neutral wall stake
		turnToAngle(0);
		driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		driveAndTurnDistanceTiles(-0.5, 0.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);
		setArmStage(1, 0.5);


		/* Score 1 ring */

		// Turn
		turnToAngle(90);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);


		/* Score 3 rings and redirect 1 ring */

		// Turn
		turnToAngle(-60);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Redirect ring
		turnToAngle(55);
		setIntakeToArm(1);
		driveAndTurnDistanceTiles(1.0, 55, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);


		/* Place mobile goal in corner */

		// Release goal and push to corner
		turnToAngle(105);
		setGoalClampState(0);
		setIntakeState(0, 1.0);
		driveAndTurnDistanceTiles(-1.0, 105, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(0.5, 135, 80.0, 100.0, defaultMoveTilesErrorRange, 1.0);
	}

	void finalSkills() {
		/* Score on alliance wall stake */

		// Turn and raise arm
		turnToAngle(160);
		setArmStage(2);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Score on alliance wall stake
		turnToAngle(-90);
		driveAndTurnDistanceTiles(1.0, -90.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.0);
		driveAndTurnDistanceTiles(-0.5, -90.0, 80.0, 100.0, defaultMoveTilesErrorRange, 0.5);


		/* Climb on ladder */

		// Turn and raise arm
		turnToAngle(145);
		setArmStage(3);

		// Follow path
		runFollowSpline();

		// Wait
		waitUntil(_pathFollowCompleted);

		// Climb
		turnToAngle(45);
		driveAndTurnDistanceTiles(1.0, 45, 40.0, 100.0);
	}
}
