#include "Pas1-Lib/Chassis/Move/global-move-to.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"


namespace {
using aespa_lib::geometry::Vector2D;
using aespa_lib::datas::Linegular;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::AutonSettings;
using pas1_lib::chassis::settings::MotionHandler;
using namespace aespa_lib::units;
using namespace pas1_lib::chassis::move::global;

namespace drive_to_pose {
void runDriveToPose();

const double carrot_distanceThreshold_tiles = 0.5;

Linegular _targetPose(0, 0, 0);
double _carrotLead;
bool _isReverseHeading;
double _maxVelocity_pct, _maxTurnVelocity_pct;
double _runTimeout_sec;
Differential *_diff_chassis;
}

}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


void driveToPose(Differential &chassis, driveToPose_params params, bool async) {
	chassis.motionHandler.incrementMotion();
	waitUntil(chassis.motionHandler.getIsInMotion() == false);

	drive_to_pose::_targetPose = Linegular(params.x.tiles(), params.y.tiles(), params.rotation);
	drive_to_pose::_carrotLead = params.carrotLead;
	drive_to_pose::_isReverseHeading = params.isReverse;
	drive_to_pose::_maxVelocity_pct = params.maxVelocity_pct;
	drive_to_pose::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	drive_to_pose::_runTimeout_sec = params.runTimeout_sec;
	drive_to_pose::_diff_chassis = &chassis;

	_isDriveToPoseSettled = false;
	_driveToPoseDistanceError = 1e9;

	if (async) {
		task asyncDrive([]() -> int {
			drive_to_pose::runDriveToPose();
			return 1;
		});
	} else {
		drive_to_pose::runDriveToPose();
	}
}

Length _driveToPoseDistanceError(0);
bool _isDriveToPoseSettled;


}
}
}
}


namespace {

namespace drive_to_pose {
void runDriveToPose() {
	// Get global variables
	Linegular targetPose = _targetPose;
	double carrotLead = _carrotLead;
	bool isReverse = _isReverseHeading;
	double maxVelocity_pct = _maxVelocity_pct;
	double maxTurnVelocity_pct = _maxTurnVelocity_pct;
	double runTimeout_sec = _runTimeout_sec;
	Differential *chassis = _diff_chassis;
	AutonSettings &autonSettings = chassis->autonSettings;
	MotionHandler &motionHandler = chassis->motionHandler;

	// Initial state
	Linegular startLg = chassis->getLookPose();

	// Error
	_driveToPoseDistanceError = (targetPose - startLg).getXYMagnitude();
	PolarAngle driveToPoseRotateError = (targetPose - startLg).getRotation();

	// Config
	const double velocityFactor = (isReverse ? -1 : 1);
	const double rotationOffset_degrees = (isReverse ? 180 : 0);


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
	printf("----- Drive to Pose (%.3f, %.3f) tiles (%.3f) deg -----\n", targetPose.getX(), targetPose.getY(), targetPose.getRotation().polarDeg());

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

		// Get carrot point
		bool isCloseToTarget = _driveToPoseDistanceError.tiles() < carrot_distanceThreshold_tiles;
		Vector2D carrotPoint = [&]() -> Vector2D {
			if (isCloseToTarget) return targetPose.getPosition();
			Vector2D leadVector = Vector2D::fromPolar(targetPose.getRotation(), carrotLead * _driveToPoseDistanceError.tiles());
			if (isReverse) return targetPose.getPosition() + leadVector;
			return targetPose.getPosition() - leadVector;
		}();

		// Get target rotation
		double targetRotation_degrees = [&]() -> double {
			if (isCloseToTarget) return targetPose.getRotation().polarDeg();
			return (carrotPoint - currentLg.getPosition()).angleFrom(Vector2D(1, 0)).polarDeg() + rotationOffset_degrees;
		}();
		targetRotation_degrees = aespa_lib::genutil::modRange(targetRotation_degrees, 360, -180);


		/* ---------- Linear ---------- */

		// Compute linear distance error
		double carrotDistanceError = (carrotPoint - currentLg.getPosition()).getMagnitude();
		double overallDistanceError = (targetPose - currentLg).getXYMagnitude();
		_driveToPoseDistanceError = std::fabs(overallDistanceError);

		// Compute motor velocity pid-value from error
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(carrotDistanceError);
		double velocity_pct;
		velocity_pct = autonSettings.distanceError_tiles_to_velocity_pct_pid.getValue();
		velocity_pct = aespa_lib::genutil::clamp(velocity_pct, -maxVelocity_pct, maxVelocity_pct);
		velocity_pct *= velocityFactor;

		// Update error patience
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(overallDistanceError));


		/* ---------- Angular ---------- */

		// Compute polar heading error
		double rotateError = targetRotation_degrees - currentLg.getRotation().polarDeg();
		rotateError = aespa_lib::genutil::modRange(rotateError, 360, -180);
		driveToPoseRotateError = rotateError;

		// Compute heading pid-value from error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError);
		double rotateVelocity_pct;
		rotateVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		rotateVelocity_pct = aespa_lib::genutil::clamp(rotateVelocity_pct, -maxTurnVelocity_pct, maxTurnVelocity_pct);


		/* Debug print */
		// printf("DIS CDE: %.3f, VLin: %.3f, TRot: %.3f, VRot: %.3f\n", carrotDistanceError, velocity_pct, targetRotation_degrees, rotateVelocity_pct);
		// printf("ANG CUR: %.3f CRT: %.3f %.3f TGT: %.3f, DE: %.3f\n", currentLg.getRotation().polarDeg(), carrotPoint.x, carrotPoint.y, targetRotation_degrees, rotateError);
		// printf("ANG CUR: %.3f TGT: %.3f, RV: %.3f\n", currentLg.getRotation().polarDeg(), targetRotation_degrees, rotateVelocity_pct);


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

	printf("Err: %.3f tiles %.3f deg\n", _driveToPoseDistanceError.tiles(), driveToPoseRotateError.polarDeg());

	// Stop
	chassis->stopMotors(brake);
	motionHandler.exitMotion();

	// Settled
	_driveToPoseDistanceError = -1;
	_isDriveToPoseSettled = true;
}
}

}
