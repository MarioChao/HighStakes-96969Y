#include "Pas1-Lib/Chassis/Move/global-move-to.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using namespace pas1_lib::chassis::move::global;

void runDriveToPoint();

const double turnTo_distanceThreshold = 0.3;

double _targetX, _targetY;
bool _isReverseHeading;
double _maxVelocity_pct, _maxTurnVelocity_pct;
double _runTimeout_sec;
Differential *_diff_chassis;
}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


void driveToPoint(Differential &chassis, driveToPoint_params params, bool async) {
	_targetX = params.x_tiles;
	_targetY = params.y_tiles;
	_isReverseHeading = params.isReverse;
	_maxVelocity_pct = params.maxVelocity_pct;
	_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	_runTimeout_sec = params.runTimeout_sec;
	_diff_chassis = &chassis;

	_isDriveToPointSettled = false;

	if (async) {
		task asyncDrive([]() -> int {
			runDriveToPoint();
			return 1;
		});
	} else {
		runDriveToPoint();
	}
}

double _linearPathDistanceError;
bool _isDriveToPointSettled;


}
}
}
}


namespace {
void runDriveToPoint() {
	// Get global variables
	double x_tiles = _targetX;
	double y_tiles = _targetY;
	bool isReverse = _isReverseHeading;
	double maxVelocity_pct = _maxVelocity_pct;
	double maxTurnVelocity_pct = _maxTurnVelocity_pct;
	double runTimeout_sec = _runTimeout_sec;
	Differential *chassis = _diff_chassis;
	AutonSettings &autonSettings = chassis->autonSettings;

	// Initial state
	Linegular startLg = chassis->getLookPose();

	// Target state
	const double targetDistance = aespa_lib::genutil::euclideanDistance({ startLg.getX(), startLg.getY() }, { x_tiles, y_tiles });
	double targetRotation_degrees = aespa_lib::genutil::toDegrees(atan2(y_tiles - startLg.getY(), x_tiles - startLg.getX()));
	_linearPathDistanceError = targetDistance;

	// Config
	const double velocityFactor = (isReverse ? -1 : 1);
	const double rotationOffset_degrees = (isReverse ? 180 : 0);

	// Reset PID
	// driveTurn_driveTargetDistance_voltPid.resetErrorToZero();
	// driveTurn_rotateTargetAngle_voltPid.resetErrorToZero();
	autonSettings.distanceError_tiles_to_velocity_pct_pid.resetErrorToZero();
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset patience
	// driveError_tilesPatience.reset();
	autonSettings.distanceError_tiles_patience.reset();

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	while (true) {
		// Check timeout
		if (runTimeout.isExpired()) {
			break;
		}

		// Check settled
		// if (driveTurn_driveTargetDistance_voltPid.isSettled() && driveTurn_rotateTargetAngle_voltPid.isSettled()) {
		if (
			autonSettings.distanceError_tiles_to_velocity_pct_pid.isSettled()
			&& autonSettings.angleError_degrees_to_velocity_pct_pid.isSettled()
		) {
			printf("Settled\n");
			break;
		}

		// Check exhausted
		// if (driveError_tilesPatience.isExhausted()) {
		if (autonSettings.distanceError_tiles_patience.isExhausted()) {
			printf("Exhausted\n");
			break;
		}

		// Get current state
		Linegular currentLg = chassis->getLookPose();
		double currentX = currentLg.getX();
		double currentY = currentLg.getY();


		/* Linear */

		// Compute linear distance error
		double travelDistance = aespa_lib::genutil::euclideanDistance({ startLg.getX(), startLg.getY() }, { currentX, currentY });
		double distanceError = targetDistance - travelDistance;
		_linearPathDistanceError = distanceError;

		// Compute motor velocity pid-value from error
		// driveTurn_driveTargetDistance_voltPid.computeFromError(distanceError);
		// driveTurn_driveTargetDistance_velocityPid.computeFromError(distanceError);
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(distanceError);
		double velocity_pct;
		velocity_pct = autonSettings.distanceError_tiles_to_velocity_pct_pid.getValue();
		velocity_pct = aespa_lib::genutil::clamp(velocity_pct, -maxVelocity_pct, maxVelocity_pct);
		velocity_pct *= velocityFactor;

		// Update error patience
		// driveError_tilesPatience.computePatience(std::fabs(distanceError));
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(distanceError));


		/* Angular */

		// Compute target polar heading
		if (distanceError > turnTo_distanceThreshold) {
			targetRotation_degrees = aespa_lib::genutil::toDegrees(std::atan2(y_tiles - currentY, x_tiles - currentX)) + rotationOffset_degrees;
		}

		// Compute polar heading error
		double rotateError = targetRotation_degrees - currentLg.getThetaPolarAngle_degrees();
		// if (autonfunctions::_useRelativeRotation) {
		if (autonSettings.useRelativeRotation) {
			rotateError = aespa_lib::genutil::modRange(rotateError, 360, -180);
		}

		// Compute heading pid-value from error
		// driveTurn_rotateTargetAngle_voltPid.computeFromError(rotateError);
		// driveTurn_rotateTargetAngle_velocityPid.computeFromError(rotateError);
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError);
		double rotateVelocity_pct;
		rotateVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		rotateVelocity_pct = aespa_lib::genutil::clamp(rotateVelocity_pct, -maxTurnVelocity_pct, maxTurnVelocity_pct);


		/* Debug print */
		// printf("DIS TR: %.3f, TGT: %.3f, DE: %.3f, VLin: %.3f, VRot: %.3f\n", travelDistance, targetDistance, distanceError, velocity_pct, rotateVelocity_pct);
		// printf("ANG CUR: %.3f, TGT: %.3f, DE: %.3f\n", currentLg.getThetaPolarAngle_degrees(), targetRotation_degrees, rotateError);


		/* Combined */

		// Drive
		chassis->control_local2d(0, velocity_pct, rotateVelocity_pct);

		// Delay
		wait(20, msec);
	}

	// Stop
	chassis->stopMotors(coast);

	// Settled
	_linearPathDistanceError = -1;
	_isDriveToPointSettled = true;
}
}
