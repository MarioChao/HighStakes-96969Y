#include "Pas1-Lib/Chassis/Move/local-move-by.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"

#include "global-vars.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using pas1_lib::chassis::settings::MotionHandler;
using namespace pas1_lib::chassis::move::local;

namespace turn_to_angle {
void runTurnToAngle();

double _targetAngle_polarDegrees;
double _maxTurnVelocity_pct;
double _centerOffset_tiles;
double _runTimeout_sec;
Differential *_diff_chassis;
}

}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace local {


void turnToAngle(Differential &chassis, turnToAngle_params params, bool async) {
	chassis.motionHandler.incrementMotion();
	waitUntil(chassis.motionHandler.getIsInMotion() == false);

	turn_to_angle::_targetAngle_polarDegrees = params.targetAngle.polarDeg();
	turn_to_angle::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	turn_to_angle::_centerOffset_tiles = params.centerOffset_tiles;
	turn_to_angle::_runTimeout_sec = params.runTimeout_sec;
	turn_to_angle::_diff_chassis = &chassis;

	_isTurnToAngleSettled = false;
	_turnAngleError_degrees = 1e9;

	if (async) {
		task asyncDrive([]() -> int {
			turn_to_angle::runTurnToAngle();
			return 1;
		});
	} else {
		turn_to_angle::runTurnToAngle();
	}
}

double _turnAngleError_degrees;
bool _isTurnToAngleSettled;


}
}
}
}


namespace {

namespace turn_to_angle {
void runTurnToAngle() {
	// Get global variables
	double targetAngle_polarDegrees = _targetAngle_polarDegrees;
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
	printf("----- Turn to %.3f deg -----\n", targetAngle_polarDegrees);

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

		// Get current robot heading
		double currentRotation_polarDegrees = chassis->getLookPose().getRotation().polarDeg();

		// Compute heading error
		double rotateError_degrees = targetAngle_polarDegrees - currentRotation_polarDegrees;
		if (autonSettings.useRelativeRotation) {
			rotateError_degrees = aespa_lib::genutil::modRange(rotateError_degrees, 360, -180);
		}
		_turnAngleError_degrees = std::fabs(rotateError_degrees);

		/* PID */

		// Compute heading pid-value from error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError_degrees);

		// Update error patience
		autonSettings.angleError_degrees_patience.computePatience(std::fabs(rotateError_degrees));

		// Compute motor rotate velocities
		double averageMotorVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		double leftMotorVelocity_pct = leftVelocityFactor * averageMotorVelocity_pct;
		double rightMotorVelocity_pct = rightVelocityFactor * averageMotorVelocity_pct;
		// printf("CUR: %.3f, TAR: %.3f, RotERR: %.3f, PID: %.3f\n", currentRotation_polarDegrees, targetAngle_polarDegrees, rotateError_degrees, averageMotorVelocity_pct);

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

	printf("Err: %.3f deg\n", _turnAngleError_degrees);

	// Stop
	chassis->stopMotors(brake);
	motionHandler.exitMotion();

	// Settled
	_turnAngleError_degrees = -1;
	_isTurnToAngleSettled = true;
}
}

}
