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
	// Correctors
	DriftCorrection driftCorrector(InertialSensor, 0, 0);

	// Controllers
	PatienceController driveError_tilesPatience(10, 0.0417, false);

	PIDController driveTurn_driveTargetDistancePid(410, 0, 40, autonvals::defaultMoveTilesErrorRange);
	PIDController driveTurn_rotateTargetAnglePid(1.0, 0.05, 0.01, autonvals::defaultTurnAngleErrorRange);

	// Constraints
	const double turnTo_distanceThreshold = 0.3;
}

namespace autonfunctions {
	void driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse, double maxVelocityPct, double maxTurnVelocityPct, double runTimeout) {
		// Set corrector
		driftCorrector.setInitial();

		// Initial state
		Linegular startLg = mainOdometry.getLookLinegular();

		// Target state
		const double targetDistance = genutil::euclideanDistance({startLg.getX(), startLg.getY()}, {x_tiles, y_tiles});
		double targetRotation_degrees = genutil::toDegrees(atan2(y_tiles - startLg.getY(), x_tiles - startLg.getX()));

		// Config
		const double velocityFactor = (isReverse ? -1 : 1);
		const double rotationOffset_degrees = (isReverse ? 180 : 0);

		// Reset PID
		driveTurn_driveTargetDistancePid.resetErrorToZero();
		driveTurn_rotateTargetAnglePid.resetErrorToZero();

		// Reset patience
		driveError_tilesPatience.reset();

		// Reset timer
		timer timeout;

		while (timeout.value() < runTimeout) {
			// Check settled
			if (driveTurn_driveTargetDistancePid.isSettled() && driveTurn_rotateTargetAnglePid.isSettled()) {
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
			// printf("TR: %.3f, TGT: %.3f, DE: %.3f\n", travelDistance, targetDistance, distanceError);

			// Compute motor velocity pid-value from error
			driveTurn_driveTargetDistancePid.computeFromError(distanceError);
			double velocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, driveTurn_driveTargetDistancePid.getValue()));
			velocityPct *= velocityFactor;

			// Update error patience
			driveError_tilesPatience.computePatience(std::fabs(distanceError));


			/* Angular */

			// Compute target polar heading
			if (std::fabs(distanceError) > turnTo_distanceThreshold) {
				targetRotation_degrees = genutil::toDegrees(atan2(y_tiles - currentY, x_tiles - currentX));
			}

			// Compute polar heading error
			double rotateError = targetRotation_degrees - currentLg.getThetaPolarAngle_degrees() + rotationOffset_degrees;
			if (_useRelativeRotation) {
				rotateError = genutil::modRange(rotateError, 360, -180);
			}

			// Compute heading pid-value from error
			driveTurn_rotateTargetAnglePid.computeFromError(rotateError);
			double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, driveTurn_rotateTargetAnglePid.getValue()));


			/* Combined */

			// Compute final motor velocities
			double leftVelocityPct = velocityPct - rotateVelocityPct;
			double rightVelocityPct = velocityPct + rotateVelocityPct;

			// Drive with velocities
			botdrive::driveVoltage(genutil::pctToVolt(leftVelocityPct), genutil::pctToVolt(rightVelocityPct), 10);

			// Delay
			task::sleep(20);
		}

		// Stop
		LeftRightMotors.stop(coast);

		// Correct
		driftCorrector.correct();
	}

	void runLinearPIDPath(std::vector<std::vector<double>> waypoints, double maxVelocity, bool isReverse) {
		Linegular lg(0, 0, 0);
		for (std::vector<double> point : waypoints) {
			// Rotation
			lg = mainOdometry.getLookLinegular();
			double angle_degrees = angle::swapFieldPolar_degrees(genutil::toDegrees(atan2(point[1] - lg.getY(), point[0] - lg.getX())));
			if (isReverse) angle_degrees += 180;
			turnToAngle(angle_degrees);

			// Linear
			lg = mainOdometry.getLookLinegular();
			// double drive_distance = genutil::euclideanDistance({lg.getX(), lg.getY()}, {point[0], point[1]}) * (isReverse ? -1 : 1);
			// printf("ST: X: %.3f, Y: %.3f, dist: %.3f\n", lg.getX(), lg.getY(), drive_distance);
			// driveAndTurnDistanceTiles(drive_distance, angle_degrees, maxVelocity);
			driveTurnToFace_tiles(point[0], point[1], isReverse);

			// Info
			lg = mainOdometry.getLookLinegular();
			printf("ED: X: %.3f, Y: %.3f\n", lg.getX(), lg.getY());
		}
	}
}