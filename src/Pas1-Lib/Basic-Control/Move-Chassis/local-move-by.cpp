#include "Pas1-Lib/Basic-Control/Move-Chassis/local-move-by.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"

#include "global-vars.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::trajectories::ConstraintSequence;
using pas1_lib::basic_control::chassis::Differential;
using pas1_lib::basic_control::chassis::BotInfo;
using pas1_lib::basic_control::chassis::AutonSettings;
using namespace pas1_lib::basic_control::move_chassis::local;

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
namespace basic_control {
namespace move_chassis {
namespace local {


void turnToAngle(chassis::Differential &chassis, turnToAngle_params params, bool async) {
	turn_to_angle::_targetAngle_polarDegrees = params.targetAngle_polarDegrees;
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


void driveAndTurn(chassis::Differential &chassis, driveAndTurn_params params, bool async) {
	drive_and_turn::_distance_tiles = params.distance_tiles;
	drive_and_turn::_targetAngle_polarDegrees = params.targetAngle_polarDegrees;
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
	double leftVelocityFactor = leftRotateRadius_tiles / averageRotateRadius_tiles;
	double rightVelocityFactor = -rightRotateRadius_tiles / averageRotateRadius_tiles;
	// L_vel = L_dist / time
	// R_vel = R_dist / time = L_vel * (R_dist / L_dist)

	// Reset PID
	// turnToAngle_rotateTargetAngleVoltPid.resetErrorToZero();
	// turnToAngle_rotateTargetAngleVelocityPctPid.resetErrorToZero();
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset patience
	// angleError_degreesPatience.reset();
	autonSettings.angleError_degrees_patience.reset();

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	// while (!turnToAngle_rotateTargetAngleVoltPid.isSettled()) {
	while (!autonSettings.angleError_degrees_to_velocity_pct_pid.isSettled()) {
		// Check timeout
		if (runTimeout.isExpired()) {
			break;
		}

		// Check exhausted
		// if (angleError_degreesPatience.isExhausted()) {
		if (autonSettings.angleError_degrees_patience.isExhausted()) {
			printf("Exhausted\n");
			break;
		}

		// printf("Inertial value: %.3f\n", InertialSensor.rotation(degrees));

		// Get current robot heading
		// double currentRotation_degrees = InertialSensor.rotation(degrees);
		double currentRotation_degrees = chassis->getLookPose().getThetaPolarAngle_degrees();

		// Compute heading error
		// double rotateError = fieldAngle_degrees - currentRotation_degrees;
		double rotateError_degrees = targetAngle_polarDegrees - currentRotation_degrees;
		if (autonSettings.useRelativeRotation) {
			rotateError_degrees = aespa_lib::genutil::modRange(rotateError_degrees, 360, -180);
		}
		_turnAngleError_degrees = std::fabs(rotateError_degrees);

		// Compute heading pid-value from error
		// turnToAngle_rotateTargetAngleVoltPid.computeFromError(rotateError);
		// turnToAngle_rotateTargetAngleVelocityPctPid.computeFromError(rotateError);
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError_degrees);

		// Update error patience
		// angleError_degreesPatience.computePatience(std::fabs(rotateError));
		autonSettings.angleError_degrees_patience.computePatience(std::fabs(rotateError_degrees));

		// Compute motor rotate velocities
		// double averageMotorVelocityPct;
		// if (useVolt) {
		// 	averageMotorVelocityPct = turnToAngle_rotateTargetAngleVoltPid.getValue();
		// } else {
		// 	averageMotorVelocityPct = turnToAngle_rotateTargetAngleVelocityPctPid.getValue();
		// }
		double averageMotorVelocity_pct = autonSettings.angleError_degrees_to_velocity_pct_pid.getValue();
		double leftMotorVelocity_pct = leftVelocityFactor * averageMotorVelocity_pct;
		double rightMotorVelocity_pct = rightVelocityFactor * averageMotorVelocity_pct;

		// Scale velocity to maximum
		// double scaleFactor = aespa_lib::genutil::getScaleFactor(maxVelocity_pct, { leftMotorVelocityPct, rightMotorVelocityPct });
		double scaleFactor = aespa_lib::genutil::getScaleFactor(maxTurnVelocity_pct, { leftMotorVelocity_pct, rightMotorVelocity_pct });
		leftMotorVelocity_pct *= scaleFactor;
		rightMotorVelocity_pct *= scaleFactor;

		// Get linear & angular
		double linearVelocity_pct = (leftMotorVelocity_pct + rightMotorVelocity_pct) / 2.0;
		double angularVelocity_pct = (rightMotorVelocity_pct - leftMotorVelocity_pct) / 2.0;

		// Drive with velocities
		chassis->control_local2d(0, linearVelocity_pct, angularVelocity_pct);

		wait(5, msec);
	}

	// Stop
	chassis->stopMotors(coast);

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
	TrajectoryPlanner motionProfile(distance_tiles, botInfo.trackWidth_tiles, 64);
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
	double maxAccel = botInfo.maxVel_tilesPerSec * 1.5;
	motionProfile.addCenterConstraint_maxMotion({ botInfo.maxVel_tilesPerSec, maxAccel });
	motionProfile.addTrackConstraint_maxMotion({ botInfo.maxVel_tilesPerSec, maxAccel });
	motionProfile.calculateMotionProfile();
	// testTrajectoryPlan = motionProfile;
	// trajectoryTestTimer.reset();

	// Reset PID
	// driveAndTurn_reachedTargetPid.resetErrorToZero();
	// driveAndTurn_drivePositionPid.resetErrorToZero();
	// driveAndTurn_driveVelocityPid.resetErrorToZero();
	// driveAndTurn_rotateTargetAnglePid.resetErrorToZero();
	// driveAndTurn_synchronizeVelocityPid.resetErrorToZero();
	autonSettings.distanceError_tiles_to_velocity_pct_pid.resetErrorToZero();
	autonSettings.fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid.resetErrorToZero();
	autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.resetErrorToZero();
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset patience
	// driveError_inchesPatience.reset();
	autonSettings.distanceError_tiles_patience.reset();

	// Reset timer
	timer runningTimer;

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	// Print info
	printf("Drive pid with trajectory of %.3f seconds\n", motionProfile.getTotalTime());

	while (true) {
		// Check timeout
		if (runTimeout.isExpired()) {
			break;
		}

		// Check profile ended
		if (runningTimer.time(seconds) >= motionProfile.getTotalTime() + 0.2) {
			break;
		}

		// Check settled
		if (autonSettings.distanceError_tiles_to_velocity_pct_pid.isSettled()) {
			break;
		}

		// Check exhausted
		if (autonSettings.distanceError_tiles_patience.isExhausted()) {
			break;
		}


		/* ---------- Linear ---------- */

		// Compute current travel distance
		double targetDistance_tiles = distance_tiles;
		Linegular currentPose = chassis->getLookPose();
		double currentTravelDistance_tiles = (currentPose - initialPose).getXYMagnitude() * aespa_lib::genutil::signum(targetDistance_tiles);
		double currentTravelVelocity_tilesPerSec = chassis->getLookVelocity();

		// Compute trajectory values
		std::pair<double, std::vector<double>> motion = motionProfile.getMotionAtTime(runningTimer.time(seconds));
		double trajAcceleration_tilesPerSec2 = motion.second[1];
		double trajVelocity_tilesPerSec = motion.second[0];
		double trajPosition_tiles = motion.first;

		/* Position feedback */

		// Compute trajectory distance error
		double trajectoryDistanceError_tiles = trajPosition_tiles - currentTravelDistance_tiles;

		// Compute travel distance error
		double targetDistanceError = targetDistance_tiles - currentTravelDistance_tiles;
		// autonfunctions::pid_diff::_driveDistanceError_inches = fabs(targetDistanceError);
		// driveAndTurn_reachedTargetPid.computeFromError(targetDistanceError);
		_driveDistanceError_tiles = std::fabs(targetDistanceError);
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(targetDistanceError);

		// Compute pid-value from error
		// driveAndTurn_drivePositionPid.computeFromError(trajectoryDistanceError_inches);
		// double positionPidVelocity_pct = driveAndTurn_drivePositionPid.getValue();
		autonSettings.fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid.computeFromError(trajectoryDistanceError_tiles);
		double positionPidVelocity_tilesPerSec = autonSettings.fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid.getValue();
		positionPidVelocity_tilesPerSec = aespa_lib::genutil::clamp(
			positionPidVelocity_tilesPerSec, -botInfo.maxVel_tilesPerSec, botInfo.maxVel_tilesPerSec
		);
		// printf("t: %.3f, trajd: %.3f, cntd; %.3f,\n", runningTimer.time(seconds), trajPosition_tiles, currentTravelDistance_inches / field::tileLengthIn);
		// printf("t: %.3f, err: %.3f, pid: %.3f\n", runningTimer.time(seconds), trajectoryDistanceError_inches, positionPidVelocity_pct);

		// Update error patience
		// driveError_inchesPatience.computePatience(std::fabs(targetDistanceError));
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(targetDistanceError));

		/* Feedforward */

		double desiredVelocity_tilesPerSec = trajVelocity_tilesPerSec;
		// desiredVelocity_tilesPerSec += positionPidVelocity_pct / 100.0 * botinfo::maxV_tilesPerSec;
		desiredVelocity_tilesPerSec += positionPidVelocity_tilesPerSec;

		// driveAndTurn_driveMotionForward.computeFromMotion(desiredVelocity_tilesPerSec, trajAcceleration_tilesPerSec2);
		// bool useS = currentTravelVelocity_inchesPerSec < 0.5;
		// double forwardVelocity_pct = aespa_lib::genutil::voltToPct(driveAndTurn_driveMotionForward.getValue(useS));
		autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.computeFromMotion(desiredVelocity_tilesPerSec, trajAcceleration_tilesPerSec2);
		bool useS = currentTravelVelocity_tilesPerSec < 0.02;
		double forwardVelocity_pct = aespa_lib::genutil::voltToPct(autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.getValue(useS));

		/* Velocity feedback */

		// Compute trajectory velocity error
		// double trajectoryVelocityError_inchesPerSec = trajVelocity_tilesPerSec * field::tileLengthIn - currentTravelVelocity_inchesPerSec;
		double trajectoryVelocityError_tilesPerSec = trajVelocity_tilesPerSec - currentTravelVelocity_tilesPerSec;

		// Compute pid-value from error
		// driveAndTurn_driveVelocityPid.computeFromError(trajectoryVelocityError_inchesPerSec);
		// double velocityPidVelocity_pct = driveAndTurn_driveVelocityPid.getValue();
		autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.computeFromError(trajectoryVelocityError_tilesPerSec);
		double velocityPidVelocity_pct = aespa_lib::genutil::voltToPct(autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.getValue());

		/* Combined */

		// Compute linear velocity
		double linearVelocity_pct = forwardVelocity_pct + velocityPidVelocity_pct;
		// linearVelocity_pct = positionPidVelocity_pct;

		// double desiredPct = desiredVelocity_tilesPerSec / botinfo::maxV_tilesPerSec * 100.0;
		// double veloError = desiredPct - currentTravelVelocity_inchesPerSec / field::tileLengthIn / botinfo::maxV_tilesPerSec * 100.0;
		// double desiredPct = desiredVelocity_tilesPerSec * botInfo.tilesPerSecond_to_pct;
		// double veloError = desiredPct - currentTravelVelocity_tilesPerSec * botInfo.tilesPerSecond_to_pct;
		// printf("velErr: %.3f tiles/sec, desired: %.3f t/s\n", veloError, desiredVelocity_tilesPerSec);
		// printf("dv: %+3.3f%, vf: %+3.3f\%, vp: %+3.3f\%, ver: %+3.3f%\n", desiredPct, forwardVelocity_pct, velocityPidVelocity_pct, veloError);


		/* ---------- Angular ---------- */

		// Get current robot heading
		double currentRotation_polarDegrees = currentPose.getThetaPolarAngle_degrees();

		// Compute heading error
		double rotateError_degrees = targetAngle_polarDegrees - currentRotation_polarDegrees;
		if (autonSettings.useRelativeRotation) {
			rotateError_degrees = aespa_lib::genutil::modRange(rotateError_degrees, 360, -180);
		}

		// Compute heading pid-value from error
		// driveAndTurn_rotateTargetAnglePid.computeFromError(rotateError_degrees);
		// double rotateVelocity_pct = aespa_lib::genutil::clamp(driveAndTurn_rotateTargetAnglePid.getValue(), -maxTurnVelocity_pct, maxTurnVelocity_pct);
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError(rotateError_degrees);
		double rotateVelocity_pct = aespa_lib::genutil::clamp(
			autonSettings.angleError_degrees_to_velocity_pct_pid.getValue(), -maxTurnVelocity_pct, maxTurnVelocity_pct
		);


		/* Combined */

		// Drive with velocities
		chassis->control_local2d(0, linearVelocity_pct, rotateVelocity_pct);

		wait(5, msec);
	}

	// Stop
	chassis->stopMotors(coast);

	// Settled
	_driveDistanceError_tiles = -1;
	_isDriveAndTurnSettled = true;
}
}

}
