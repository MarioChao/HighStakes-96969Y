#include "Autonomous/autonPaths.h"

void autonpaths::runAutonTest() {
	setRobotRotation(0.0);

	setDifferentialUseRelativeRotation(false);

	if (false) {
		robotChassis.setLookPose(aespa_lib::datas::Linegular(1.5, 0.5, 90));
		
		pas1_lib::planning::profiles::SplineProfile *splineProfile = splineProfile_storage.getStored("test").get();
		double startAngle_degrees = splineProfile->spline.getLinegularAt(0, splineProfile->willReverse).getThetaPolarAngle_degrees();
		local::turnToAngle(robotChassis, local::turnToAngle_params(startAngle_degrees), false);
		follow::followPath(robotChassis, follow::followPath_params(splineProfile), true);
		waitUntil(follow::_isPathFollowCompleted);
	}

	if (false) {
		robotChassis.setLookPose(aespa_lib::datas::Linegular(0, 0, 90));

		global::driveToPoint(robotChassis, global::driveToPoint_params(0, 1, false), true);
		waitUntil(global::_isDriveToPointSettled);
		global::driveToPoint(robotChassis, global::driveToPoint_params(0, 0, true), true);
		waitUntil(global::_isDriveToPointSettled);

		local::driveAndTurn(robotChassis, local::driveAndTurn_params(1, 90, 75), true);
		waitUntil(local::_isDriveAndTurnSettled);
		local::driveAndTurn(robotChassis, local::driveAndTurn_params(-1, 90, {{0, 20}, {0.5, 60}}), true);
		waitUntil(local::_isDriveAndTurnSettled);

		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 100, 0), true);
		waitUntil(local::_isTurnToAngleSettled);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 50, robotChassis.botInfo.trackWidth_tiles / 2.0), true);
		waitUntil(local::_isTurnToAngleSettled);
		local::turnToAngle(robotChassis, local::turnToAngle_params(180, 100, 0), true);
		waitUntil(local::_isTurnToAngleSettled);
		local::turnToAngle(robotChassis, local::turnToAngle_params(90, 50, -robotChassis.botInfo.trackWidth_tiles / 2.0), true);
		waitUntil(local::_isTurnToAngleSettled);
	}

	if (true) {
		robotChassis.setLookPose(aespa_lib::datas::Linegular(0, 0, 90));

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
		// async_driveAndTurnDistance_qtInches(400, 0.0, {{0, 50}});
		// waitUntil(_isDriveAndTurnSettled);
		async_driveAndTurnDistance_qtInches(200, 0.0, {{0, 100}, {100, 40}});
		waitUntil(_isDriveAndTurnSettled);
		async_driveAndTurnDistance_qtInches(-200, 0.0, {{0, 40}, {100, 100}});
		waitUntil(_isDriveAndTurnSettled);
		// async_driveAndTurnDistance_qtInches(100, 0.0, 100);
		// waitUntil(_isDriveAndTurnSettled);
		// async_driveAndTurnDistance_qtInches(100, 0.0, 40);
		// waitUntil(_isDriveAndTurnSettled);
		// async_driveAndTurnDistance_qtInches(-100, 0.0, 40);
		// waitUntil(_isDriveAndTurnSettled);
		// async_driveAndTurnDistance_qtInches(-100, 0.0, 100);
		// waitUntil(_isDriveAndTurnSettled);
	}

	if (false) {
		driveAndTurnDistanceTiles(1.0, 0.0, 50.0, 100.0, 6.0);
		driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, 3.0);
		driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, 3.0);
		driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, 3.0);
		driveAndTurnDistanceTiles(2.0, 0.0, 100.0, 100.0, 3.0);
		driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 100.0, 3.0);
	}

	if (false) {
		turnToAngle(45);
		task::sleep(200);
		turnToAngleVelocity(-45, 15);
		task::sleep(200);
		turnToAngleVelocity(90, 30);
		task::sleep(200);
		turnToAngleVelocity(-90, 60);
		task::sleep(200);
		turnToAngle(180);
		task::sleep(200);
		turnToAngle(-180);
		task::sleep(200);
		turnToAngle(450);
		task::sleep(200);
		turnToAngle(-450, 0.0, 3.5);
		task::sleep(200);
		turnToAngle(0);
	}

	setDifferentialUseRelativeRotation(true);

	mainOdometry.setPosition_scaled(0, 0);
	setRobotRotation(0);

	if (false) {
		runLinearPIDPath({
			{0, 1}, {1, 1}, {1, 0}, {0, 0}
		}, 100);
	}

	if (false) {
		runLinearPIDPath({{0, 1}}, 100);
		runLinearPIDPath({{0, 2}}, 100);
		runLinearPIDPath({{0, 1}}, 100, true);
		runLinearPIDPath({{0, 0}}, 100, true);
		runLinearPIDPath({{0, 2}}, 100);
		runLinearPIDPath({{0, 0}}, 100, true);
		// turnToAngle(0);
	}

	if (true) {
		setArmResetDefaultStage(2);
	}
}

void autonpaths::odometryRadiusTest() {
	setRobotRotation(0.0);

	wait(100, msec);

	printf("Clockwise\n");

	mainOdometry.printDebug();
	turnToAngleVelocity(360.0 * 10.0, 30.0, 0.0, 40.0);
	mainOdometry.printDebug();

	wait(1, sec);

	setRobotRotation(0.0);

	wait(100, msec);

	printf("Counter clockwise\n");

	mainOdometry.printDebug();
	turnToAngleVelocity(-360.0 * 10.0, 30.0, 0.0, 40.0);
	mainOdometry.printDebug();
}
