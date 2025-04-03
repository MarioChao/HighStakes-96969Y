#include "Pas1-Lib/Chassis/Move/global-move-to.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using namespace pas1_lib::chassis::move::global;

namespace turn_to_face {
void runTurnToFace();

double _targetX, _targetY;
bool _isReverse;
double _maxTurnVelocity_pct;
double _centerOffset_tiles;
double _runTimeout_sec;
Differential *_diff_chassis;
}

namespace drive_to_point {
void runDriveToPoint();

const double turnTo_distanceThreshold = 0.3;

double _targetX, _targetY;
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


void turnToFace(Differential &chassis, turnToFace_params params, bool async) {
	turn_to_face::_targetX = params.x.tiles();
	turn_to_face::_targetY = params.y.tiles();
	turn_to_face::_isReverse = params.isReverse;
	turn_to_face::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	turn_to_face::_centerOffset_tiles = params.centerOffset_tiles;
	turn_to_face::_runTimeout_sec = params.runTimeout_sec;
	turn_to_face::_diff_chassis = &chassis;

	_isTurnToFaceSettled = false;

	if (async) {
		task asyncDrive([]() -> int {
			turn_to_face::runTurnToFace();
			return 1;
		});
	} else {
		turn_to_face::runTurnToFace();
	}
}

double _turnToFaceError_degrees;
bool _isTurnToFaceSettled;


void driveToPoint(Differential &chassis, driveToPoint_params params, bool async) {
	drive_to_point::_targetX = params.x.tiles();
	drive_to_point::_targetY = params.y.tiles();
	drive_to_point::_isReverseHeading = params.isReverse;
	drive_to_point::_maxVelocity_pct = params.maxVelocity_pct;
	drive_to_point::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	drive_to_point::_runTimeout_sec = params.runTimeout_sec;
	drive_to_point::_diff_chassis = &chassis;

	_isDriveToPointSettled = false;

	if (async) {
		task asyncDrive([]() -> int {
			drive_to_point::runDriveToPoint();
			return 1;
		});
	} else {
		drive_to_point::runDriveToPoint();
	}
}

double _linearPathDistanceError;
bool _isDriveToPointSettled;


}
}
}
}


namespace {

namespace turn_to_face {
void runTurnToFace() {
	// Get global variables
	double x_tiles = _targetX;
	double y_tiles = _targetY;
	bool isReverse = _isReverse;
	double maxTurnVelocity_pct = _maxTurnVelocity_pct;
	double centerOffset_tiles = _centerOffset_tiles;
	double runTimeout_sec = _runTimeout_sec;
	Differential *chassis = _diff_chassis;
	BotInfo &botInfo = chassis->botInfo;
	AutonSettings &autonSettings = chassis->autonSettings;

	// Center of rotations
	double leftRotateRadius_tiles = botInfo.trackWidth_tiles / 2.0 + centerOffset_tiles;
	double rightRotateRadius_tiles = botInfo.trackWidth_tiles / 2.0 - centerOffset_tiles;
	double averageRotateRadius_tiles = (leftRotateRadius_tiles + rightRotateRadius_tiles) / 2;

	// Velocity factors
	double leftVelocityFactor = -leftRotateRadius_tiles / averageRotateRadius_tiles;
	double rightVelocityFactor = rightRotateRadius_tiles / averageRotateRadius_tiles;
	// L_vel = L_dist / time
	// R_vel = R_dist / time = L_vel * (R_dist / L_dist)

	// Config
	const double rotationOffset_degrees = (isReverse ? 180 : 0);

	// Reset PID
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset slew
	autonSettings.linearAcceleration_pctPerSec_slew.reset();
	autonSettings.angularAcceleration_pctPerSec_slew.reset();

	// Reset patience
	autonSettings.angleError_degrees_patience.reset();

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	while (true) {
		// Check timeout
		if (runTimeout.isExpired()) {
			printf("Expired\n");
			break;
		}

		// Check settled
		if (autonSettings.angleError_degrees_to_velocity_pct_pid.isSettled()) {
			printf("Settled\n");
			break;
		}

		// Check exhausted
		if (autonSettings.angleError_degrees_patience.isExhausted()) {
			printf("Exhausted\n");
			break;
		}

		/* ---------- Angular ---------- */

		// Get current robot pose
		Linegular robotLg = chassis->getLookPose();

		// Get current robot heading
		double currentRotation_degrees = robotLg.getRotation().polarDeg();

		// Compute target heading
		double targetAngle_polarDegrees = aespa_lib::genutil::toDegrees(atan2(y_tiles - robotLg.getY(), x_tiles - robotLg.getX())) + rotationOffset_degrees;

		// Compute heading error
		double rotateError_degrees = targetAngle_polarDegrees - currentRotation_degrees;
		rotateError_degrees = aespa_lib::genutil::modRange(rotateError_degrees, 360, -180);
		_turnToFaceError_degrees = std::fabs(rotateError_degrees);
		
		/* PID */

		// Compute heading pid-value from error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError_degrees);

		// Update error patience
		autonSettings.angleError_degrees_patience.computePatience(std::fabs(rotateError_degrees));

		// Compute motor rotate velocities
		double averageMotorVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		double leftMotorVelocity_pct = leftVelocityFactor * averageMotorVelocity_pct;
		double rightMotorVelocity_pct = rightVelocityFactor * averageMotorVelocity_pct;

		// Scale velocity overshoot
		double scaleFactor = aespa_lib::genutil::getScaleFactor(maxTurnVelocity_pct, { leftMotorVelocity_pct, rightMotorVelocity_pct });
		leftMotorVelocity_pct *= scaleFactor;
		rightMotorVelocity_pct *= scaleFactor;

		// Get linear & angular
		double linearVelocity_pct = (leftMotorVelocity_pct + rightMotorVelocity_pct) / 2.0;
		double angularVelocity_pct = (rightMotorVelocity_pct - leftMotorVelocity_pct) / 2.0;

		// Slew
		autonSettings.linearAcceleration_pctPerSec_slew.computeFromTarget(linearVelocity_pct);
		autonSettings.angularAcceleration_pctPerSec_slew.computeFromTarget(angularVelocity_pct);
		linearVelocity_pct = autonSettings.linearAcceleration_pctPerSec_slew.getValue();
		angularVelocity_pct = autonSettings.angularAcceleration_pctPerSec_slew.getValue();

		// Drive with velocities
		chassis->control_local2d(0, linearVelocity_pct, angularVelocity_pct);

		wait(10, msec);
	}

	// Stop
	chassis->stopMotors(coast);

	// Settled
	_turnToFaceError_degrees = -1;
	_isTurnToFaceSettled = true;
}
}

namespace drive_to_point {
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

	// Reset slew
	autonSettings.linearAcceleration_pctPerSec_slew.reset();
	autonSettings.angularAcceleration_pctPerSec_slew.reset();

	// Reset patience
	// driveError_tilesPatience.reset();
	autonSettings.distanceError_tiles_patience.reset();

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	while (true) {
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

		// Get current state
		Linegular currentLg = chassis->getLookPose();
		double currentX = currentLg.getX();
		double currentY = currentLg.getY();


		/* ---------- Linear ---------- */

		// Compute linear distance error
		double travelDistance = aespa_lib::genutil::euclideanDistance({ startLg.getX(), startLg.getY() }, { currentX, currentY });
		double distanceError = targetDistance - travelDistance;
		_linearPathDistanceError = std::fabs(distanceError);

		// Compute motor velocity pid-value from error
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(distanceError);
		double velocity_pct;
		velocity_pct = autonSettings.distanceError_tiles_to_velocity_pct_pid.getValue();
		velocity_pct = aespa_lib::genutil::clamp(velocity_pct, -maxVelocity_pct, maxVelocity_pct);
		velocity_pct *= velocityFactor;

		// Update error patience
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(distanceError));


		/* ---------- Angular ---------- */

		// Compute target polar heading
		if (distanceError > turnTo_distanceThreshold) {
			targetRotation_degrees = aespa_lib::genutil::toDegrees(std::atan2(y_tiles - currentY, x_tiles - currentX)) + rotationOffset_degrees;
		}

		// Compute polar heading error
		double rotateError = targetRotation_degrees - currentLg.getRotation().polarDeg();
		rotateError = aespa_lib::genutil::modRange(rotateError, 360, -180);

		// Compute heading pid-value from error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError);
		double rotateVelocity_pct;
		rotateVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		rotateVelocity_pct = aespa_lib::genutil::clamp(rotateVelocity_pct, -maxTurnVelocity_pct, maxTurnVelocity_pct);


		/* Debug print */
		// printf("DIS TR: %.3f, TGT: %.3f, DE: %.3f, VLin: %.3f, VRot: %.3f\n", travelDistance, targetDistance, distanceError, velocity_pct, rotateVelocity_pct);
		// printf("ANG CUR: %.3f, TGT: %.3f, DE: %.3f\n", currentLg.getRotation().polarDeg(), targetRotation_degrees, rotateError);


		/* ---------- Combined ---------- */

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
		chassis->control_local2d(0, velocity_pct, rotateVelocity_pct);

		// Delay
		wait(10, msec);
	}

	// Stop
	chassis->stopMotors(coast);

	// Settled
	_linearPathDistanceError = -1;
	_isDriveToPointSettled = true;
}
}

}
