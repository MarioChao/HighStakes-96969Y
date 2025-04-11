#include "Pas1-Lib/Chassis/Move/local-move-by.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"

#include "global-vars.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::trajectories::ConstraintSequence;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using namespace pas1_lib::chassis::move::local;

namespace turn_to_angle {
void runTurnToAngle();

double _targetAngle_polarDegrees;
double _maxTurnVelocity_pct;
double _centerOffset_tiles;
double _runTimeout_sec;
Differential *_diff_chassis;
}

namespace drive_and_turn {
void runDriveAndTurn();

double _distance_tiles, _targetAngle_polarDegrees;
std::vector<std::pair<double, double>> _velocityConstraint_tiles_pct;
double _maxTurnVelocity_pct;
double _runTimeout_sec;
Differential *_diff_chassis;
}

}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace local {


void turnToAngle(Differential &chassis, turnToAngle_params params, bool async) {
	turn_to_angle::_targetAngle_polarDegrees = params.targetAngle.polarDeg();
	turn_to_angle::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	turn_to_angle::_centerOffset_tiles = params.centerOffset_tiles;
	turn_to_angle::_runTimeout_sec = params.runTimeout_sec;
	turn_to_angle::_diff_chassis = &chassis;

	_isTurnToAngleSettled = false;

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


void driveAndTurn(Differential &chassis, driveAndTurn_params params, bool async) {
	drive_and_turn::_distance_tiles = params.distance.tiles();
	drive_and_turn::_targetAngle_polarDegrees = params.targetAngle.polarDeg();
	drive_and_turn::_velocityConstraint_tiles_pct = params.velocityConstraint_tiles_pct;
	drive_and_turn::_maxTurnVelocity_pct = params.maxTurnVelocity_pct;
	drive_and_turn::_runTimeout_sec = params.runTimeout_sec;
	drive_and_turn::_diff_chassis = &chassis;

	_isDriveAndTurnSettled = false;

	if (async) {
		task asyncDrive([]() -> int {
			drive_and_turn::runDriveAndTurn();
			return 1;
		});
	} else {
		drive_and_turn::runDriveAndTurn();
	}
}

double _driveDistanceError_tiles;
bool _isDriveAndTurnSettled;


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

	// Print info
	printf("----- Turn to %.3f deg -----\n", targetAngle_polarDegrees);

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
		chassis->control_local2d(0, linearVelocity_pct, angularVelocity_pct);

		wait(10, msec);
	}

	printf("Err: %.3f deg\n", _turnAngleError_degrees);

	// Stop
	chassis->stopMotors(brake);

	// Settled
	_turnAngleError_degrees = -1;
	_isTurnToAngleSettled = true;
}
}

namespace drive_and_turn {
void runDriveAndTurn() {
	// Get global variables
	double distance_tiles = _distance_tiles;
	double targetAngle_polarDegrees = _targetAngle_polarDegrees;
	std::vector<std::pair<double, double>> velocityConstraint_tiles_pct = _velocityConstraint_tiles_pct;
	double maxTurnVelocity_pct = _maxTurnVelocity_pct;
	double runTimeout_sec = _runTimeout_sec;
	Differential *chassis = _diff_chassis;
	BotInfo &botInfo = chassis->botInfo;
	AutonSettings &autonSettings = chassis->autonSettings;

	// Variables
	Linegular initialPose = chassis->getLookPose();

	// Motion planner
	TrajectoryPlanner motionProfile(distance_tiles, botInfo.trackWidth_tiles, 0.05);
	ConstraintSequence constraintSequence;
	for (int i = 0; i < (int) velocityConstraint_tiles_pct.size(); i++) {
		auto constraint = velocityConstraint_tiles_pct[i];
		constraintSequence.addConstraints({
			{
				constraint.first,
				{
					aespa_lib::genutil::clamp(constraint.second, 1, 100) / 100.0 * botInfo.maxVel_tilesPerSec
				}
			}
		});
	}
	motionProfile.addCenterConstraintSequence(constraintSequence);
	motionProfile.addCenterConstraint_maxMotion({ botInfo.maxVel_tilesPerSec, botInfo.maxAccel_tilesPerSec2 * 0.85 });
	motionProfile.addTrackConstraint_maxMotion({ botInfo.maxVel_tilesPerSec, botInfo.maxAccel_tilesPerSec2 * 0.85 });
	motionProfile.calculateMotionProfile();

	// Trajectory graph
	testTrajectoryPlan = motionProfile;
	trajectoryTestTimer.reset();

	// Reset PID
	// driveAndTurn_synchronizeVelocityPid.resetErrorToZero();
	autonSettings.distanceError_tiles_to_velocity_pct_pid.resetErrorToZero();
	autonSettings.fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid.resetErrorToZero();
	autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.resetErrorToZero();
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset slew
	autonSettings.linearAcceleration_pctPerSec_slew.reset();
	autonSettings.angularAcceleration_pctPerSec_slew.reset();

	// Reset patience
	autonSettings.distanceError_tiles_patience.reset();

	// Reset timer
	timer runningTimer;

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	// Print info
	printf("----- Drive pid trajectory %.3f sec -----\n", motionProfile.getTotalTime());

	while (true) {
		/* ---------- End conditions ---------- */

		// Check timeout
		if (runTimeout.isExpired()) {
			printf("Expired\n");
			break;
		}

		// Check profile ended
		if (runningTimer.time(seconds) >= motionProfile.getTotalTime() + 0.2) {
			printf("Profile ended\n");
			break;
		}

		// Check settled
		if (autonSettings.distanceError_tiles_to_velocity_pct_pid.isSettled()) {
			printf("Settled\n");
			break;
		}

		// Check exhausted
		if (autonSettings.distanceError_tiles_patience.isExhausted()) {
			printf("Exhausted\n");
			break;
		}


		/* ---------- Linear ---------- */

		// Compute current values
		double targetDistance_tiles = distance_tiles;
		Linegular currentPose = chassis->getLookPose();
		double currentTravelDistance_tiles = (currentPose - initialPose).getXYMagnitude() * aespa_lib::genutil::signum(targetDistance_tiles);
		double currentTravelVelocity_tilesPerSec = chassis->getLookVelocity();

		// Compute trajectory values
		std::pair<double, std::vector<double>> motion = motionProfile.getMotionAtTime(runningTimer.time(seconds));
		double trajAcceleration_tilesPerSec2 = motion.second[1];
		double trajVelocity_tilesPerSec = motion.second[0];
		double trajPosition_tiles = motion.first;

		/* Overall error */

		// Compute travel distance error
		double targetDistanceError = targetDistance_tiles - currentTravelDistance_tiles;
		_driveDistanceError_tiles = std::fabs(targetDistanceError);
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(targetDistanceError);

		// Update error patience
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(targetDistanceError));
		// printf("DERR: %.3f %.3f %.3f\n", targetDistanceError, currentTravelDistance_tiles, targetDistance_tiles);

		/* Feedforward + feedback */

		// Position feedback
		autonSettings.fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid.computeFromError(trajPosition_tiles - currentTravelDistance_tiles);
		double positionPidVelocity_tilesPerSec = autonSettings.fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid.getValue();

		// Motion feedforward
		double desiredVelocity_tilesPerSec = trajVelocity_tilesPerSec + positionPidVelocity_tilesPerSec;
		autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.computeFromMotion(desiredVelocity_tilesPerSec, trajAcceleration_tilesPerSec2);

		// Velocity feedback
		autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.computeFromError(desiredVelocity_tilesPerSec - currentTravelVelocity_tilesPerSec);

		// Combined
		bool useS = std::fabs(currentTravelVelocity_tilesPerSec) < 0.02;
		double forwardVelocity_volt = autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.getValue(useS);
		double feedbackVelocity_volt = autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.getValue();
		double linearVelocity_pct = aespa_lib::genutil::voltToPct(forwardVelocity_volt + feedbackVelocity_volt);
		// printf("DV %.3f %.3f %.3f %.3f %.3f\n", targetDistanceError, currentTravelVelocity_tilesPerSec, desiredVelocity_tilesPerSec, forwardVelocity_volt, feedbackVelocity_volt);


		/* ---------- Angular ---------- */

		// Get current robot heading
		double currentRotation_polarDegrees = currentPose.getRotation().polarDeg();

		// Compute heading error
		double rotateError_degrees = targetAngle_polarDegrees - currentRotation_polarDegrees;
		if (autonSettings.useRelativeRotation) {
			rotateError_degrees = aespa_lib::genutil::modRange(rotateError_degrees, 360, -180);
		}

		// Compute heading pid-value from error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError_degrees);
		double rotateVelocity_pct = aespa_lib::genutil::clamp(
			autonSettings.angleError_degrees_to_velocity_pct_pid.getValue(), -maxTurnVelocity_pct, maxTurnVelocity_pct
		);


		/* ---------- Combined ---------- */

		// Scale velocity overshoot
		double leftVelocity_pct = linearVelocity_pct - rotateVelocity_pct;
		double rightVelocity_pct = linearVelocity_pct + rotateVelocity_pct;
		double scaleFactor = aespa_lib::genutil::getScaleFactor(100.0, { leftVelocity_pct, rightVelocity_pct });
		leftVelocity_pct *= scaleFactor;
		rightVelocity_pct *= scaleFactor;
		linearVelocity_pct = (leftVelocity_pct + rightVelocity_pct) / 2.0;
		rotateVelocity_pct = (rightVelocity_pct - leftVelocity_pct) / 2.0;

		// Slew
		autonSettings.linearAcceleration_pctPerSec_slew.computeFromTarget(linearVelocity_pct);
		autonSettings.angularAcceleration_pctPerSec_slew.computeFromTarget(rotateVelocity_pct);
		linearVelocity_pct = autonSettings.linearAcceleration_pctPerSec_slew.getValue();
		rotateVelocity_pct = autonSettings.angularAcceleration_pctPerSec_slew.getValue();

		// Drive with velocities
		chassis->control_local2d(0, linearVelocity_pct, rotateVelocity_pct);

		wait(10, msec);
	}

	printf("Err: %.3f tiles\n", _driveDistanceError_tiles);

	// Stop
	chassis->stopMotors(brake);

	// Settled
	_driveDistanceError_tiles = -1;
	_isDriveAndTurnSettled = true;
}
}

}
