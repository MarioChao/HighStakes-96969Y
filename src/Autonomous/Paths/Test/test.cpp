#include "Autonomous/autonPaths.h"

void autonpaths::runAutonTest() {
	setRobotPosition(1.5, 0.5);
	setRobotRotation(0.0);

	setDifferentialUseRelativeRotation(false);

	if (false) {
		setRobotLookPose(aespa_lib::datas::Linegular(1.5, 0.5, 90));

		pathbuild::runFollowSpline(robotChassis, "test");
		waitUntil(follow::_isPathFollowCompleted);

		global::turnToFace(robotChassis, global::turnToFace_params(1.5, 0.5), false);
		global::driveToPoint(robotChassis, global::driveToPoint_params(1.5, 0.5), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90), false);
	}

	if (true) {
		setRobotLookPose(aespa_lib::datas::Linegular(1, 0, 90));

		global::driveToPoint(robotChassis, global::driveToPoint_params(1, 1, false), false);
		global::driveToPoint(robotChassis, global::driveToPoint_params(1, 0, true), false);

		local::driveAndTurn(robotChassis, local::driveAndTurn_params(1, 90, 75), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(-1, 90, { {0, 20}, {0.5, 60} }), false);

		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 100, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 50, botInfo.trackWidth_tiles / 2.0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 100, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 50, -botInfo.trackWidth_tiles / 2.0), false);
	}

	if (false) {
		setRobotLookPose(aespa_lib::datas::Linegular(0, 0, 90));

		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 100, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(270, 100, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 50, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 50, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(270, 100, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(0, 50, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 100, 0), false);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 100, 0), false);
	}

	if (false) {
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(200_qtIn, 0, { {0, 100}, {(100_qtIn).tiles(), 40} }), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(-200_qtIn, 0, { {0, 40}, {(100_qtIn).tiles(), 100} }), false);
	}

	if (false) {
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(1, 0, 50), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(1, 0, 100), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(-1, 0, 100), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(-1, 0, 100), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(2, 0, 100), false);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(-2, 0, 100), false);
	}

	if (false) {
		local::turnToAngle(robotChassis, local::turnToAngle_params(45), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(-45, 15), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 30), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(-90, 60), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(180), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(-180), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(450), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(-450), false);
		task::sleep(200);
		local::turnToAngle(robotChassis, local::turnToAngle_params(0), false);
	}

	setDifferentialUseRelativeRotation(true);

	setRobotRotation(0);

	if (false) {
		runLinearPIDPath({
			{0, 1}, {1, 1}, {1, 0}, {0, 0}
		}, 100);
	}

	if (false) {
		runLinearPIDPath({ {1_tiles, 1_tiles} }, 100);
		runLinearPIDPath({ {1_tiles, 2_tiles} }, 100);
		runLinearPIDPath({ {1_tiles, 1_tiles} }, 100, true);
		runLinearPIDPath({ {1_tiles, 0_tiles} }, 100, true);
		runLinearPIDPath({ {1_tiles, 2_tiles} }, 100);
		runLinearPIDPath({ {1_tiles, 0_tiles} }, 100, true);
	}

	if (false) {
		setArmResetDefaultStage(2);
	}
}

void autonpaths::odometryRadiusTest() {
	setRobotRotation(0.0);

	wait(100, msec);

	printf("Clockwise\n");

	mainOdometry.printDebug();
	local::turnToAngle(robotChassis, local::turnToAngle_params(3600, 30, 0, 40.0), false);
	mainOdometry.printDebug();

	wait(1, sec);

	setRobotRotation(0.0);

	wait(100, msec);

	printf("Counter clockwise\n");

	mainOdometry.printDebug();
	local::turnToAngle(robotChassis, local::turnToAngle_params(-3600, 30, 0, 40.0), false);
	mainOdometry.printDebug();
}
