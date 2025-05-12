#include "Pas1-Lib/Chassis/Move/global-move-to.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::AutonSettings;
using pas1_lib::chassis::settings::MotionHandler;
using namespace aespa_lib::units;
using namespace pas1_lib::chassis::move::global;

namespace drive_to_point {
void runDriveToPoint();

const double turnTo_distanceThreshold_tiles = 0.3;

double _targetX, _targetY;
bool _isReverseHeading;
double _earlyStopOffset_tiles;
double _maxVelocity_pct, _maxTurnVelocity_pct;
double _runTimeout_sec;
Differential *_diff_chassis;
}

}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


void driveToPoint(Differential &chassis, driveToPoint_params params, bool async) {
	chassis.motionHandler.incrementMotion();
	waitUntil(chassis.motionHandler.getIsInMotion() == false);

	drive_to_point::_targetX = params.x.tiles();
	drive_to_point::_targetY = params.y.tiles();
	drive_to_point::_isReverseHeading = params.isReverse;
	drive_to_point::_earlyStopOffset_tiles = params.earlyStopOffset.tiles();
	drive_to_point::_maxVelocity_pct = params.maxVelocity_pct;
	drive_to_point::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	drive_to_point::_runTimeout_sec = params.runTimeout_sec;
	drive_to_point::_diff_chassis = &chassis;

	_isDriveToPointSettled = false;
	_driveToPointDistanceError = 1e9;
	_driveToPointAngleError_degrees = 1e9;

	if (async) {
		task asyncDrive([]() -> int {
			drive_to_point::runDriveToPoint();
			return 1;
		});
	} else {
		drive_to_point::runDriveToPoint();
	}
}

double _driveToPointAngleError_degrees;
Length _driveToPointDistanceError(0);
bool _isDriveToPointSettled;


}
}
}
}


namespace {

namespace drive_to_point {
void runDriveToPoint() {
	// Get global variables
	double x_tiles = _targetX;
	double y_tiles = _targetY;
	bool isReverse = _isReverseHeading;
	double earlyStopOffset_tiles = _earlyStopOffset_tiles;
	double maxVelocity_pct = _maxVelocity_pct;
	double maxTurnVelocity_pct = _maxTurnVelocity_pct;
	double runTimeout_sec = _runTimeout_sec;
	Differential *chassis = _diff_chassis;
	AutonSettings &autonSettings = chassis->autonSettings;
	MotionHandler &motionHandler = chassis->motionHandler;

	// Initial state
	Linegular startLg = chassis->getLookPose();

	// Target state
	const double targetDistance_tiles = aespa_lib::genutil::euclideanDistance({ startLg.getX(), startLg.getY() }, { x_tiles, y_tiles });
	_driveToPointDistanceError = targetDistance_tiles;

	// Config
	const double velocityFactor = (isReverse ? -1 : 1);
	const double rotationOffset_degrees = (isReverse ? 180 : 0);

	// Previous states
	double previousTargetRotation_degrees = aespa_lib::genutil::toDegrees(std::atan2(y_tiles - startLg.getY(), x_tiles - startLg.getX())) + rotationOffset_degrees;
	_driveToPointAngleError_degrees = std::fabs(aespa_lib::genutil::modRange(previousTargetRotation_degrees - startLg.getRotation().polarDeg(), 360, -180));


	// Reset PID
	autonSettings.distanceError_tiles_to_velocity_pct_pid.resetErrorToZero();
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset slew
	autonSettings.linearAcceleration_pctPerSec_slew.reset();
	autonSettings.angularAcceleration_pctPerSec_slew.reset();

	// Reset patience
	autonSettings.distanceError_tiles_patience.reset();

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	// Store motion id
	motionHandler.enterMotion();
	int currentMotionId = motionHandler.getMotionId();

	// Print info
	printf("----- Drive to (%.3f, %.3f) tiles -----\n", x_tiles, y_tiles);

	while (true) {
		/* ---------- End conditions ---------- */

		// Check motion id
		if (!motionHandler.isRunningMotionId(currentMotionId)) {
			printf("Motion cancelled\n");
			motionHandler.exitMotion();
			return;
		}

		// Check timeout
		if (runTimeout.isExpired()) {
			printf("Expired\n");
			break;
		}

		// Check settled
		if (
			autonSettings.distanceError_tiles_to_velocity_pct_pid.isSettled()
			&& autonSettings.angleError_degrees_to_velocity_pct_pid.isSettled()
		) {
			printf("Settled\n");
			break;
		}

		// Check exhausted
		if (autonSettings.distanceError_tiles_patience.isExhausted()) {
			printf("Exhausted\n");
			break;
		}


		/* ---------- States ---------- */

		// Get current state
		Linegular currentLg = chassis->getLookPose();
		double currentX = currentLg.getX();
		double currentY = currentLg.getY();

		// Get target rotation
		bool isCloseToTarget = _driveToPointDistanceError.tiles() < turnTo_distanceThreshold_tiles;
		double targetRotation_degrees = [&]() -> double {
			if (isCloseToTarget) return previousTargetRotation_degrees;
			// if (isCloseToTarget) return currentLg.getRotation().polarDeg();
			return aespa_lib::genutil::toDegrees(std::atan2(y_tiles - currentY, x_tiles - currentX)) + rotationOffset_degrees;
		}();
		previousTargetRotation_degrees = targetRotation_degrees;


		/* ---------- Linear ---------- */

		// Compute linear distance error
		double travelDistance_tiles = aespa_lib::genutil::euclideanDistance({ startLg.getX(), startLg.getY() }, { currentX, currentY });
		double distanceError_tiles = targetDistance_tiles - travelDistance_tiles - earlyStopOffset_tiles;
		_driveToPointDistanceError = std::fabs(distanceError_tiles);

		// Compute motor velocity pid-value from error
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(distanceError_tiles);
		double velocity_pct;
		velocity_pct = autonSettings.distanceError_tiles_to_velocity_pct_pid.getValue();
		velocity_pct = aespa_lib::genutil::clamp(velocity_pct, -maxVelocity_pct, maxVelocity_pct);
		velocity_pct *= velocityFactor;

		// Update error patience
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(distanceError_tiles));


		/* ---------- Angular ---------- */

		// Compute polar heading error
		double rotateError = targetRotation_degrees - currentLg.getRotation().polarDeg();
		rotateError = aespa_lib::genutil::modRange(rotateError, 360, -180);
		_driveToPointAngleError_degrees = std::fabs(rotateError);

		// Compute heading pid-value from error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError);
		double rotateVelocity_pct;
		rotateVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		rotateVelocity_pct = aespa_lib::genutil::clamp(rotateVelocity_pct, -maxTurnVelocity_pct, maxTurnVelocity_pct);


		/* Debug print */
		// printf("DIS TR: %.3f, TGT: %.3f, DE: %.3f, VLin: %.3f, VRot: %.3f\n", travelDistance_tiles, targetDistance_tiles, distanceError_tiles, velocity_pct, rotateVelocity_pct);
		// printf("ANG CUR: %.3f, TGT: %.3f, DE: %.3f\n", currentLg.getRotation().polarDeg(), targetRotation_degrees, rotateError);


		/* ---------- Combined ---------- */

		// Cosine trick https://www.ctrlaltftc.com/practical-examples/drivetrain-control#cosine-trick
		double cosine_value = std::cos(aespa_lib::units::operator ""_polarDeg((long double) rotateError).polarRad());
		velocity_pct *= cosine_value;

		// Reset distance patience if turning a lot
		if (std::fabs(rotateError) > 15) autonSettings.distanceError_tiles_patience.reset();

		// Scale velocity overshoot
		double leftVelocity_pct = velocity_pct - rotateVelocity_pct;
		double rightVelocity_pct = velocity_pct + rotateVelocity_pct;
		double scaleFactor = aespa_lib::genutil::getScaleFactor(100.0, { leftVelocity_pct, rightVelocity_pct });
		leftVelocity_pct *= scaleFactor;
		rightVelocity_pct *= scaleFactor;
		velocity_pct = (leftVelocity_pct + rightVelocity_pct) / 2.0;
		rotateVelocity_pct = (rightVelocity_pct - leftVelocity_pct) / 2.0;

		// Slew
		autonSettings.linearAcceleration_pctPerSec_slew.computeFromTarget(velocity_pct);
		autonSettings.angularAcceleration_pctPerSec_slew.computeFromTarget(rotateVelocity_pct);
		velocity_pct = autonSettings.linearAcceleration_pctPerSec_slew.getValue();
		rotateVelocity_pct = autonSettings.angularAcceleration_pctPerSec_slew.getValue();

		// Drive
		chassis->control_local2d(0, velocity_pct, rotateVelocity_pct, true);

		// Delay
		wait(10, msec);
	}

	printf("Err: %.3f tiles\n", _driveToPointDistanceError.tiles());

	// Stop
	chassis->stopMotors(brake);
	motionHandler.exitMotion();

	// Settled
	_driveToPointAngleError_degrees = -1;
	_driveToPointDistanceError = -1;
	_isDriveToPointSettled = true;
}
}

}
