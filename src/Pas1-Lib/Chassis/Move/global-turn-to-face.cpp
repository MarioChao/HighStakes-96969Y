#include "Pas1-Lib/Chassis/Move/global-move-to.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using pas1_lib::chassis::settings::MotionHandler;
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

}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


void turnToFace(Differential &chassis, turnToFace_params params, bool async) {
	chassis.motionHandler.incrementMotion();
	waitUntil(chassis.motionHandler.getIsInMotion() == false);

	turn_to_face::_targetX = params.x.tiles();
	turn_to_face::_targetY = params.y.tiles();
	turn_to_face::_isReverse = params.isReverse;
	turn_to_face::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	turn_to_face::_centerOffset_tiles = params.centerOffset_tiles;
	turn_to_face::_runTimeout_sec = params.runTimeout_sec;
	turn_to_face::_diff_chassis = &chassis;

	_isTurnToFaceSettled = false;
	_turnToFaceError_degrees = 1e9;

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
	MotionHandler &motionHandler = chassis->motionHandler;

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

	// Store motion id
	motionHandler.enterMotion();
	int currentMotionId = motionHandler.getMotionId();

	// Print info
	printf("----- Turn to face (%.3f, %.3f) tiles -----\n", x_tiles, y_tiles);

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
		double targetAngle_polarDegrees = aespa_lib::genutil::toDegrees(std::atan2(y_tiles - robotLg.getY(), x_tiles - robotLg.getX())) + rotationOffset_degrees;

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
		chassis->control_local2d(0, linearVelocity_pct, angularVelocity_pct, true);

		wait(10, msec);
	}

	printf("Err: %.3f deg\n", _turnToFaceError_degrees);

	// Stop
	chassis->stopMotors(brake);
	motionHandler.exitMotion();

	// Settled
	_turnToFaceError_degrees = -1;
	_isTurnToFaceSettled = true;
}
}

}
