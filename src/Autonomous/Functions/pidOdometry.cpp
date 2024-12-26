#include "Autonomous/autonFunctions.h"

#include "AutonUtilities/driftCorrection.h"
#include "AutonUtilities/pidController.h"
#include "AutonUtilities/linegular.h"
#include "AutonUtilities/patienceController.h"

#include "Mechanics/botDrive.h"

#include "Utilities/angleUtility.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "Utilities/generalUtility.h"

namespace {
	using namespace autonfunctions::driveturn;

	// Correctors
	DriftCorrection driftCorrector(InertialSensor, 0, 0);

	// Controllers
	PatienceController driveError_tilesPatience(4, 0.01, false);

	PIDController driveTurn_driveTargetDistance_voltPid(90, 0, 6, autonvals::defaultMoveTilesErrorRange, 3);
	PIDController driveTurn_rotateTargetAngle_voltPid(2.0, 0, 0, autonvals::defaultTurnAngleErrorRange, 3);

	PIDController driveTurn_driveTargetDistance_velocityPid(70, 0, 0, autonvals::defaultMoveTilesErrorRange);
	PIDController driveTurn_rotateTargetAngle_velocityPid(0.3, 0.0, 0.03, autonvals::defaultTurnAngleErrorRange);

	bool useVolt = true;

	// Constraints
	const double turnTo_distanceThreshold = 0.3;

	// Function
	void runDriveTurnToFace();
}

namespace autonfunctions {
	namespace driveturn {
		void async_driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout) {
			_linearPathDistanceError = 1e9;
			_targetX = x_tiles;
			_targetY = y_tiles;
			_isReverseHeading = isReverse;
			_maxVelocity_pct = maxVelocity_pct;
			_maxTurnVelocity_pct = maxTurnVelocity_pct;
			_runTimeout = runTimeout;
			_isDriveTurnSettled = false;
			task driveTurn([] () -> int {
				runDriveTurnToFace();
				return 1;
			});
		}

		void driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout) {
			async_driveTurnToFace_tiles(x_tiles, y_tiles, isReverse, maxVelocity_pct, maxTurnVelocity_pct, runTimeout);
			waitUntil(_isDriveTurnSettled);
		}

		double _linearPathDistanceError;
		double _targetX, _targetY;
		bool _isReverseHeading;
		double _maxVelocity_pct, _maxTurnVelocity_pct;
		double _runTimeout;

		bool _isDriveTurnSettled;
	}

	void turnToFace_tiles(double x_tiles, double y_tiles, bool isReverse, double maxTurnVelocity_pct) {
		Linegular lg = mainOdometry.getLookLinegular();
		double angle_degrees = angle::swapFieldPolar_degrees(genutil::toDegrees(atan2(y_tiles - lg.getY(), x_tiles - lg.getX())));
		if (isReverse) angle_degrees += 180;
		turnToAngleVelocity(angle_degrees, maxTurnVelocity_pct);
	}

	void runLinearPIDPath(std::vector<std::vector<double>> waypoints, double maxVelocity, bool isReverse) {
		Linegular lg(0, 0, 0);
		for (std::vector<double> point : waypoints) {
			// Rotation
			turnToFace_tiles(point[0], point[1], isReverse);

			// Linear
			lg = mainOdometry.getLookLinegular();
			// double drive_distance = genutil::euclideanDistance({lg.getX(), lg.getY()}, {point[0], point[1]}) * (isReverse ? -1 : 1);
			// printf("ST: X: %.3f, Y: %.3f, dist: %.3f\n", lg.getX(), lg.getY(), drive_distance);
			// driveAndTurnDistanceTiles(drive_distance, angle_degrees, maxVelocity);
			driveturn::driveTurnToFace_tiles(point[0], point[1], isReverse, maxVelocity);

			// Info
			lg = mainOdometry.getLookLinegular();
			printf("ED: X: %.3f, Y: %.3f\n", lg.getX(), lg.getY());
		}
	}
}

namespace {
	void runDriveTurnToFace() {
		// Get global variables
		double x_tiles = _targetX;
		double y_tiles = _targetY;
		bool isReverse = _isReverseHeading;
		double maxVelocity_pct = _maxVelocity_pct;
		double maxTurnVelocity_pct = _maxTurnVelocity_pct;
		double runTimeout = _runTimeout;

		// Set corrector
		driftCorrector.setInitial();

		// Initial state
		Linegular startLg = mainOdometry.getLookLinegular();

		// Target state
		const double targetDistance = genutil::euclideanDistance({startLg.getX(), startLg.getY()}, {x_tiles, y_tiles});
		double targetRotation_degrees = genutil::toDegrees(atan2(y_tiles - startLg.getY(), x_tiles - startLg.getX()));
		_linearPathDistanceError = targetDistance;

		// Config
		const double velocityFactor = (isReverse ? -1 : 1);
		const double rotationOffset_degrees = (isReverse ? 180 : 0);

		// Reset PID
		driveTurn_driveTargetDistance_voltPid.resetErrorToZero();
		driveTurn_rotateTargetAngle_voltPid.resetErrorToZero();

		// Reset patience
		driveError_tilesPatience.reset();

		// Reset timer
		timer timeout;

		while (timeout.value() < runTimeout) {
			// Check settled
			if (driveTurn_driveTargetDistance_voltPid.isSettled() && driveTurn_rotateTargetAngle_voltPid.isSettled()) {
				printf("Settled\n");
				break;
			}

			// Check exhausted
			if (driveError_tilesPatience.isExhausted()) {
				break;
			}

			// Get current state
			Linegular currentLg = mainOdometry.getLookLinegular();
			double currentX = currentLg.getX();
			double currentY = currentLg.getY();


			/* Linear */

			// Compute linear distance error
			double travelDistance = genutil::euclideanDistance({startLg.getX(), startLg.getY()}, {currentX, currentY});
			double distanceError = targetDistance - travelDistance;
			_linearPathDistanceError = distanceError;

			// Compute motor velocity pid-value from error
			driveTurn_driveTargetDistance_voltPid.computeFromError(distanceError);
			driveTurn_driveTargetDistance_velocityPid.computeFromError(distanceError);
			double velocity_pct;
			if (useVolt) {
				velocity_pct = driveTurn_driveTargetDistance_voltPid.getValue();
			} else {
				velocity_pct = driveTurn_driveTargetDistance_velocityPid.getValue();
			}
			velocity_pct = genutil::clamp(velocity_pct, -maxVelocity_pct, maxVelocity_pct);
			velocity_pct *= velocityFactor;

			// Update error patience
			driveError_tilesPatience.computePatience(std::fabs(distanceError));


			/* Angular */

			// Compute target polar heading
			if (distanceError > turnTo_distanceThreshold) {
				targetRotation_degrees = genutil::toDegrees(std::atan2(y_tiles - currentY, x_tiles - currentX)) + rotationOffset_degrees;
			}

			// Compute polar heading error
			double rotateError = targetRotation_degrees - currentLg.getThetaPolarAngle_degrees();
			if (autonfunctions::_useRelativeRotation) {
				rotateError = genutil::modRange(rotateError, 360, -180);
			}

			// Compute heading pid-value from error
			driveTurn_rotateTargetAngle_voltPid.computeFromError(rotateError);
			driveTurn_rotateTargetAngle_velocityPid.computeFromError(rotateError);
			double rotateVelocity_pct;
			if (useVolt) {
				rotateVelocity_pct = driveTurn_rotateTargetAngle_voltPid.getValue();
			} else {
				rotateVelocity_pct = driveTurn_rotateTargetAngle_velocityPid.getValue();
			}
			rotateVelocity_pct = genutil::clamp(rotateVelocity_pct, -maxTurnVelocity_pct, maxTurnVelocity_pct);


			/* Debug print */
			// printf("DIS TR: %.3f, TGT: %.3f, DE: %.3f, VLin: %.3f, VRot: %.3f\n", travelDistance, targetDistance, distanceError, velocity_pct, rotateVelocity_pct);
			// printf("ANG CUR: %.3f, TGT: %.3f, DE: %.3f\n", currentLg.getThetaPolarAngle_degrees(), targetRotation_degrees, rotateError);


			/* Combined */

			// Compute final motor velocities
			double leftVelocity_pct = velocity_pct - rotateVelocity_pct;
			double rightVelocity_pct = velocity_pct + rotateVelocity_pct;

			// Drive with velocities
			if (useVolt) {
				botdrive::driveVoltage(genutil::pctToVolt(leftVelocity_pct), genutil::pctToVolt(rightVelocity_pct), 12);
			} else {
				botdrive::driveVelocity(leftVelocity_pct, rightVelocity_pct);
			}

			// Delay
			task::sleep(20);
		}

		// Stop
		LeftRightMotors.stop(coast);

		// Correct
		driftCorrector.correct();

		// Settled
		_linearPathDistanceError = 0;
		_isDriveTurnSettled = true;
	}
}
