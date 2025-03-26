#include "Autonomous/autonFunctions.h"
#include "Autonomous/autonPaths.h"

#include "Pas1-Lib/Auton/Control-Loops/pid.h"
#include "Pas1-Lib/Auton/Control-Loops/feedforward.h"
#include "Pas1-Lib/Auton/End-Conditions/patience.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"

#include "AutonUtilities/driftCorrection.h"

#include "Mechanics/botDrive.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"

#include "Pas1-Lib/Planning/Trajectories/trajectory-planner.h"

#include "Simulation/robotSimulator.h"

#include "global-vars.h"
#include "main.h"

namespace {

using aespa_lib::datas::Linegular;

using pas1_lib::auton::control_loops::ForwardController;
using pas1_lib::auton::control_loops::PIDController;
using pas1_lib::auton::end_conditions::PatienceController;

using pas1_lib::planning::trajectories::ConstraintSequence;
using pas1_lib::planning::trajectories::TrajectoryPlanner;

std::vector<double> getMotorRevolutions();
double getAverageDifference(std::vector<double> vector1, std::vector<double> vector2);

bool useOdometryForPid = true;
bool useRotationSensorForPid = false;
bool useEncoderForPid = false;

// DriftCorrection driftCorrector(InertialSensor, -3.276, 3.651);
DriftCorrection driftCorrector(InertialSensor, 0, 0);

// Some controllers
PatienceController angleError_degreesPatience(40, 0.5, false);
// PatienceController driveError_inchesPatience(1000, 0.5, false);
PatienceController driveError_inchesPatience(40, 0.5, false);

PIDController turnToAngle_rotateTargetAngleVoltPid(2.3, 0.0, 0.16, autonvals::defaultTurnAngleErrorRange);
PIDController turnToAngle_rotateTargetAngleVelocityPctPid(0.4, 0.0, 0.03, autonvals::defaultTurnAngleErrorRange);

PIDController driveAndTurn_reachedTargetPid(0, 0, 0, autonvals::defaultMoveWithInchesErrorRange);
// PIDController driveAndTurn_drivePositionPid(17, 0, 0.5, autonvals::defaultMoveWithInchesErrorRange);
PIDController driveAndTurn_drivePositionPid(17, 0, 0, autonvals::defaultMoveWithInchesErrorRange); // in to pct
PIDController driveAndTurn_driveVelocityPid(1.8, 0, 0.005); // in/s to pct
ForwardController driveAndTurn_driveMotionForward(1.0, 3.1875, 0.4); // t/s to volt
PIDController driveAndTurn_rotateTargetAnglePid(1.0, 0, 0, autonvals::defaultTurnAngleErrorRange);
PIDController driveAndTurn_synchronizeVelocityPid(0.4, 0, 0, 5.0);

// Drive and turn
namespace drive_turn {

void setVariables(
	double distance_in, double targetRotation_deg,
	std::vector<std::pair<double, double>> velocityConstraint_inch_pct, double maxTurnVelocity_pct,
	double runTimeout_sec
);
void driveAndTurnDistance_inches();
double _distance_in, _fieldAngle_degrees;
std::vector<std::pair<double, double>> _velocityConstraint_inch_pct;
double _maxTurnVelocity_pct;
double _runTimeout_sec;

}

}

namespace autonfunctions {
namespace pid_diff {

/* PID differential*/

void turnToAngle(double fieldAngle_degrees, double rotateCenterOffset_inches, double runTimeout_sec) {
	turnToAngleVelocity(fieldAngle_degrees, 90.0, rotateCenterOffset_inches, runTimeout_sec);
}

/// @brief Turn the robot to face a specified angle.
/// @param fieldAngle_degrees The target angle to face in degrees.
/// @param maxVelocity_pct Maximum velocity of the fieldAngle_degrees.
/// @param rotateCenterOffset_inches The offset of the center of rotation.
/// @param runTimeout_sec Maximum seconds the function will run for.
void turnToAngleVelocity(double fieldAngle_degrees, double maxVelocity_pct, double rotateCenterOffset_inches, double runTimeout_sec) {
	// Set corrector
	driftCorrector.setInitial();

	// Center of rotations
	double leftRotateRadiusIn = botinfo::halfRobotLengthIn + rotateCenterOffset_inches;
	double rightRotateRadiusIn = botinfo::halfRobotLengthIn - rotateCenterOffset_inches;
	double averageRotateRadiusIn = (leftRotateRadiusIn + rightRotateRadiusIn) / 2;

	// Velocity factors
	double leftVelocityFactor = leftRotateRadiusIn / averageRotateRadiusIn;
	double rightVelocityFactor = -rightRotateRadiusIn / averageRotateRadiusIn;
	// L_vel = L_dist / time
	// R_vel = R_dist / time = L_vel * (R_dist / L_dist)

	// Set stopping
	LeftRightMotors.setStopping(brake);

	// Volt config
	bool useVolt = maxVelocity_pct > 25.0;

	// Reset PID
	turnToAngle_rotateTargetAngleVoltPid.resetErrorToZero();
	turnToAngle_rotateTargetAngleVelocityPctPid.resetErrorToZero();

	// Reset patience
	angleError_degreesPatience.reset();

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	while (!turnToAngle_rotateTargetAngleVoltPid.isSettled()) {
		// Check timeout
		if (runTimeout.isExpired()) {
			break;
		}

		// Check exhausted
		if (angleError_degreesPatience.isExhausted()) {
			break;
		}

		// printf("Inertial value: %.3f\n", InertialSensor.rotation(degrees));

		// Get current robot heading
		double currentRotation_degrees = InertialSensor.rotation(degrees);
		if (mainUseSimulator) {
			currentRotation_degrees = aespa_lib::angle::swapFieldPolar_degrees(aespa_lib::genutil::toDegrees(robotSimulator.angularPosition));
		}

		// Compute heading error
		double rotateError = fieldAngle_degrees - currentRotation_degrees;
		if (_useRelativeRotation) {
			rotateError = aespa_lib::genutil::modRange(rotateError, 360, -180);
		}

		// Compute heading pid-value from error
		turnToAngle_rotateTargetAngleVoltPid.computeFromError(rotateError);
		turnToAngle_rotateTargetAngleVelocityPctPid.computeFromError(rotateError);

		// Update error patience
		angleError_degreesPatience.computePatience(std::fabs(rotateError));

		// Compute motor rotate velocities
		double averageMotorVelocityPct;
		if (useVolt) {
			averageMotorVelocityPct = turnToAngle_rotateTargetAngleVoltPid.getValue();
		} else {
			averageMotorVelocityPct = turnToAngle_rotateTargetAngleVelocityPctPid.getValue();
		}
		double leftMotorVelocityPct = leftVelocityFactor * averageMotorVelocityPct;
		double rightMotorVelocityPct = rightVelocityFactor * averageMotorVelocityPct;

		// Scale velocity to maximum
		double scaleFactor = aespa_lib::genutil::getScaleFactor(maxVelocity_pct, { leftMotorVelocityPct, rightMotorVelocityPct });
		leftMotorVelocityPct *= scaleFactor;
		rightMotorVelocityPct *= scaleFactor;

		// Drive with velocities
		if (mainUseSimulator) {
			robotSimulator.angularVelocity = (rightMotorVelocityPct - leftMotorVelocityPct) / 2 / 100 * botinfo::maxV_tilesPerSec / (botinfo::halfRobotLengthIn / field::tileLengthIn);
		} else if (useVolt) {
			botdrive::driveVoltage(aespa_lib::genutil::pctToVolt(leftMotorVelocityPct), aespa_lib::genutil::pctToVolt(rightMotorVelocityPct), 10);
		} else {
			botdrive::driveVelocity(leftMotorVelocityPct, rightMotorVelocityPct);
		}

		wait(5, msec);
	}

	// Stop
	LeftRightMotors.stop(brake);

	if (mainUseSimulator) {
		robotSimulator.angularVelocity = 0;
	}

	// Correct
	driftCorrector.correct();
}

/// @brief Drive straight in the direction of the robot for a specified tile distance.
/// @param distance_tiles Distance in units of tiles.
/// @param maxVelocity_pct Maximum velocity of the drive. (can > 100)
/// @param runTimeout_sec Maximum seconds the function will run for.
void driveDistanceTiles(double distance_tiles, double maxVelocity_pct, double runTimeout_sec) {
	driveAndTurnDistanceTiles(distance_tiles, InertialSensor.rotation(), maxVelocity_pct, 100.0, runTimeout_sec);
}

/// @brief Drive the robot for a specified tile distance and rotate it to a specified rotation in degrees.
/// @param distance_tiles Distance in units of tiles.
/// @param fieldAngle_degrees The target angle to face in degrees.
/// @param maxVelocity_pct Maximum velocity of the drive. (can > 100)
/// @param maxTurnVelocity_pct Maximum rotational velocity of the drive. (can > 100)
/// @param runTimeout_sec Maximum seconds the function will run for.
void driveAndTurnDistanceTiles(double distance_tiles, double fieldAngle_degrees, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	driveAndTurnDistanceWithInches(distance_tiles * field::tileLengthIn, fieldAngle_degrees, maxVelocity_pct, maxTurnVelocity_pct, runTimeout_sec);
}

/// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
/// @param distance_inches Distance in units of inches.
/// @param fieldAngle_degrees The target angle to face in degrees.
/// @param maxVelocity_pct Maximum velocity of the drive. (can > 100)
/// @param maxTurnVelocity_pct Maximum rotational velocity of the drive. (can > 100)
/// @param runTimeout_sec Maximum seconds the function will run for.
void driveAndTurnDistanceWithInches(double distance_inches, double fieldAngle_degrees, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	async_driveAndTurnDistance_inches(distance_inches, fieldAngle_degrees, maxVelocity_pct, maxTurnVelocity_pct, runTimeout_sec);
	waitUntil(_isDriveAndTurnSettled);
}

void async_driveAndTurnDistance_tiles(double distance_tiles, double fieldAngle_degrees, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	async_driveAndTurnDistance_inches(distance_tiles * field::tileLengthIn, fieldAngle_degrees, maxVelocity_pct, maxTurnVelocity_pct, runTimeout_sec);
}

void async_driveAndTurnDistance_qtInches(double distance_qtInches, double fieldAngle_degrees, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	async_driveAndTurnDistance_inches(distance_qtInches / 4, fieldAngle_degrees, maxVelocity_pct, maxTurnVelocity_pct, runTimeout_sec);
}

void async_driveAndTurnDistance_qtInches(double distance_qtInches, double fieldAngle_degrees, std::vector<std::pair<double, double>> velocityConstraint_qtInch_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	for (int i = 0; i < (int) velocityConstraint_qtInch_pct.size(); i++) {
		velocityConstraint_qtInch_pct[i].first /= 4;
	}
	async_driveAndTurnDistance_inches(distance_qtInches / 4, fieldAngle_degrees, velocityConstraint_qtInch_pct, maxTurnVelocity_pct, runTimeout_sec);
}

void async_driveAndTurnDistance_inches(double distance_inches, double fieldAngle_degrees, double maxVelocity_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	async_driveAndTurnDistance_inches(distance_inches, fieldAngle_degrees, { {0, maxVelocity_pct} }, maxTurnVelocity_pct, runTimeout_sec);
}

void async_driveAndTurnDistance_inches(double distance_inches, double fieldAngle_degrees, std::vector<std::pair<double, double>> velocityConstraint_inch_pct, double maxTurnVelocity_pct, double runTimeout_sec) {
	// State variables
	_driveDistanceError_inches = 1e9;
	_isDriveAndTurnSettled = false;

	// Drive and turn
	drive_turn::setVariables(distance_inches, fieldAngle_degrees, velocityConstraint_inch_pct, maxTurnVelocity_pct, runTimeout_sec);
	task driveTurnTask([]() -> int {
		drive_turn::driveAndTurnDistance_inches();
		return 1;
	});
}

double _driveDistanceError_inches;
bool _isDriveAndTurnSettled;

}
}


namespace {

/// @brief Returns the current encoder readings of each chassis motor
std::vector<double> getMotorRevolutions() {
	std::vector<double> ret = {
		LeftMotorA.position(rev),
		LeftMotorB.position(rev),
		LeftMotorC.position(rev),
		RightMotorA.position(rev),
		RightMotorB.position(rev),
		RightMotorC.position(rev),
	};
	return ret;
}

/// @brief Returns the average value of vector2[i] - vector1[i].
double getAverageDifference(std::vector<double> vector1, std::vector<double> vector2) {
	int vectorSize = std::min((int) vector1.size(), (int) vector2.size());
	double totalDifference = 0;
	for (int i = 0; i < vectorSize; i++) {
		double difference = vector2[i] - vector1[i];
		totalDifference += difference;
	}
	double averageDifference = totalDifference / vectorSize;
	return averageDifference;
}

namespace drive_turn {

void setVariables(
	double distance_in, double targetRotation_deg,
	std::vector<std::pair<double, double>> velocityConstraint_inch_pct, double maxTurnVelocity_pct,
	double runTimeout_sec
) {
	_distance_in = distance_in;
	_fieldAngle_degrees = targetRotation_deg;
	_velocityConstraint_inch_pct = velocityConstraint_inch_pct;
	_maxTurnVelocity_pct = maxTurnVelocity_pct;
	_runTimeout_sec = runTimeout_sec;
}

void driveAndTurnDistance_inches() {
	// Variables
	double distance_inches = _distance_in;
	double fieldAngle_degrees = _fieldAngle_degrees;
	std::vector<std::pair<double, double>> velocityConstraint_inch_pct = _velocityConstraint_inch_pct;
	double maxTurnVelocity_pct = _maxTurnVelocity_pct;
	double runTimeout_sec = _runTimeout_sec;

	// Set corrector
	driftCorrector.setInitial();

	// Variables
	Linegular initialPose = mainOdometry.getLookPose_scaled();
	std::vector<double> initRevolutions = getMotorRevolutions();
	double lookEncoderInitialRevolution = LookEncoder.rotation(rev);
	double lookRotationInitialRevolution = LookRotation.position(rev);
	Vector3 initalSimulatorPosition = robotSimulator.position;

	// Motion planner
	TrajectoryPlanner motionProfile(distance_inches / field::tileLengthIn);
	ConstraintSequence constraintSequence;
	for (int i = 0; i < (int) velocityConstraint_inch_pct.size(); i++) {
		auto constraint = velocityConstraint_inch_pct[i];
		constraintSequence.addConstraints({
			{
				constraint.first / field::tileLengthIn,
				{
					aespa_lib::genutil::clamp(constraint.second, 1, 100) / 100.0 * autonpaths::pathbuild::maxVel_tilesPerSec
				}
			}
		});
	}
	motionProfile.addCenterConstraintSequence(constraintSequence);
	motionProfile.addCenterConstraint_maxMotion({autonpaths::pathbuild::maxVel_tilesPerSec, autonpaths::pathbuild::maxAccel});
	motionProfile.addTrackConstraint_maxMotion({autonpaths::pathbuild::maxVel_tilesPerSec, autonpaths::pathbuild::maxAccel});
	motionProfile.calculateMotionProfile();
	autonfunctions::_trajectoryPlan = motionProfile;
	autonfunctions::_splinePathTimer.reset();

	// Reset PID
	driveAndTurn_reachedTargetPid.resetErrorToZero();
	driveAndTurn_drivePositionPid.resetErrorToZero();
	driveAndTurn_driveVelocityPid.resetErrorToZero();
	driveAndTurn_rotateTargetAnglePid.resetErrorToZero();
	driveAndTurn_synchronizeVelocityPid.resetErrorToZero();

	// Reset patience
	driveError_inchesPatience.reset();

	// Reset timer
	timer runningTimer;

	// Create timeout
	pas1_lib::auton::end_conditions::Timeout runTimeout(runTimeout_sec);

	// Print info
	printf("Drive pid with trajectory of %.3f seconds\n", motionProfile.getTotalTime());

	while (!(
		driveAndTurn_reachedTargetPid.isSettled() &&
		driveAndTurn_drivePositionPid.isSettled() &&
		driveAndTurn_rotateTargetAnglePid.isSettled() &&
		runningTimer.time(seconds) > motionProfile.getTotalTime()
		)) {
		// Check timeout
		if (runTimeout.isExpired()) {
			break;
		}

		// Check profile ended
		if (runningTimer.time(seconds) >= motionProfile.getTotalTime() + 0.2) {
			break;
		}

		// Check exhausted
		if (driveError_inchesPatience.isExhausted()) {
			break;
		}


		/* Linear */

		// Compute distances
		double targetDistanceInches = distance_inches;
		double currentTravelDistance_inches = -1;
		double currentTravelVelocity_inchesPerSec = -1;
		if (mainUseSimulator) {
			double travelDistance_tiles = (robotSimulator.position - initalSimulatorPosition).getMagnitude() * aespa_lib::genutil::signum(targetDistanceInches);
			currentTravelDistance_inches = travelDistance_tiles * field::tileLengthIn;
		} else if (useOdometryForPid) {
			// Compute current travel distance in inches
			Linegular currentPose = mainOdometry.getLookPose_scaled();
			currentTravelDistance_inches = (currentPose - initialPose).getXYMagnitude() * field::tileLengthIn * aespa_lib::genutil::signum(targetDistanceInches);

			double averageTravelVel = (LeftMotors.velocity(rpm) + RightMotors.velocity(rpm)) * 0.5;
			currentTravelVelocity_inchesPerSec = averageTravelVel * (1 / 60.0) * (botinfo::driveWheelCircumIn  / botinfo::driveWheelMotorGearRatio);
		} else if (useRotationSensorForPid) {
			// Compute current travel distance in inches
			double lookCurrentRevolution = LookRotation.position(rev) - lookRotationInitialRevolution;
			currentTravelDistance_inches = lookCurrentRevolution * (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0);
			currentTravelVelocity_inchesPerSec = LookRotation.velocity(rpm) * (1 / 60.0) * (botinfo::trackingLookWheelCircumIn / botinfo::trackingLookWheelSensorGearRatio);
		} else if (useEncoderForPid) {
			// Compute current travel distance in inches
			double lookEncoderCurrentRevolution = LookEncoder.rotation(rev) - lookEncoderInitialRevolution;
			currentTravelDistance_inches = lookEncoderCurrentRevolution * (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0);
			currentTravelVelocity_inchesPerSec = LookEncoder.velocity(rpm) * (1 / 60.0) * (botinfo::trackingLookWheelCircumIn / botinfo::trackingLookWheelSensorGearRatio);
		} else {
			// Compute average traveled motor revolutions
			std::vector<double> travelRevolutions = getMotorRevolutions();
			double averageTravelRev = getAverageDifference(initRevolutions, travelRevolutions);
			double averageTravelVel = (LeftMotors.velocity(rpm) + RightMotors.velocity(rpm)) * 0.5;

			// Convert revolutions into inches
			currentTravelDistance_inches = averageTravelRev * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);
			currentTravelVelocity_inchesPerSec = averageTravelVel * (1 / 60.0) * (botinfo::driveWheelCircumIn  / botinfo::driveWheelMotorGearRatio);
		}

		// Compute trajectory values
		std::pair<double, std::vector<double>> motion = motionProfile.getMotionAtTime(runningTimer.time(seconds));
		double trajAcceleration_tilesPerSec2 = motion.second[1];
		double trajVelocity_tilesPerSec = motion.second[0];
		double trajPosition_tiles = motion.first;

		/* Position feedback */

		// Compute trajectory distance error
		double trajectoryDistanceError_inches = trajPosition_tiles * field::tileLengthIn - currentTravelDistance_inches;

		// Compute travel distance error
		double targetDistanceError = targetDistanceInches - currentTravelDistance_inches;
		autonfunctions::pid_diff::_driveDistanceError_inches = fabs(targetDistanceError);
		driveAndTurn_reachedTargetPid.computeFromError(targetDistanceError);

		// Compute pid-value from error
		driveAndTurn_drivePositionPid.computeFromError(trajectoryDistanceError_inches);
		double positionPidVelocity_pct = driveAndTurn_drivePositionPid.getValue();
		positionPidVelocity_pct = aespa_lib::genutil::clamp(positionPidVelocity_pct, -100, 100);
		// printf("t: %.3f, trajd: %.3f, cntd; %.3f,\n", runningTimer.time(seconds), trajPosition_tiles, currentTravelDistance_inches / field::tileLengthIn);
		// printf("t: %.3f, err: %.3f, pid: %.3f\n", runningTimer.time(seconds), trajectoryDistanceError_inches, positionPidVelocity_pct);

		// Update error patience
		driveError_inchesPatience.computePatience(std::fabs(targetDistanceError));

		/* Feedforward */

		double desiredVelocity_tilesPerSec = trajVelocity_tilesPerSec;
		desiredVelocity_tilesPerSec += positionPidVelocity_pct / 100.0 * botinfo::maxV_tilesPerSec;

		driveAndTurn_driveMotionForward.computeFromMotion(desiredVelocity_tilesPerSec, trajAcceleration_tilesPerSec2);
		bool useS = currentTravelVelocity_inchesPerSec < 0.5;
		double forwardVelocity_pct = aespa_lib::genutil::voltToPct(driveAndTurn_driveMotionForward.getValue(useS));

		/* Velocity feedback */

		// Compute trajectory velocity error
		double trajectoryVelocityError_inchesPerSec = trajVelocity_tilesPerSec * field::tileLengthIn - currentTravelVelocity_inchesPerSec;

		// Compute pid-value from error
		driveAndTurn_driveVelocityPid.computeFromError(trajectoryVelocityError_inchesPerSec);
		double velocityPidVelocity_pct = driveAndTurn_driveVelocityPid.getValue();

		/* Combined */

		// Compute linear velocity
		double linearVelocity_pct = forwardVelocity_pct + velocityPidVelocity_pct;
		// linearVelocity_pct = positionPidVelocity_pct;

		double desiredPct = desiredVelocity_tilesPerSec / botinfo::maxV_tilesPerSec * 100.0;
		double veloError = desiredPct - currentTravelVelocity_inchesPerSec / field::tileLengthIn / botinfo::maxV_tilesPerSec * 100.0;
		// printf("velErr: %.3f tiles/sec, desired: %.3f t/s\n", veloError, desiredVelocity_tilesPerSec);
		// printf("dv: %+3.3f%, vf: %+3.3f\%, vp: %+3.3f\%, ver: %+3.3f%\n", desiredPct, forwardVelocity_pct, velocityPidVelocity_pct, veloError);


		/* Angular */

		// Get current robot heading
		double currentRotation_degrees = InertialSensor.rotation(degrees);
		if (mainUseSimulator) currentRotation_degrees = aespa_lib::angle::swapFieldPolar_degrees(aespa_lib::genutil::toDegrees(robotSimulator.angularPosition));

		// Compute heading error
		double rotateError = fieldAngle_degrees - currentRotation_degrees;
		if (autonfunctions::_useRelativeRotation) {
			rotateError = aespa_lib::genutil::modRange(rotateError, 360, -180);
		}

		// Compute heading pid-value from error
		driveAndTurn_rotateTargetAnglePid.computeFromError(rotateError);
		double rotateVelocity_pct = aespa_lib::genutil::clamp(driveAndTurn_rotateTargetAnglePid.getValue(), -maxTurnVelocity_pct, maxTurnVelocity_pct);


		/* Combined */

		// Compute final motor velocities
		double leftVelocity_pct = linearVelocity_pct + rotateVelocity_pct;
		double rightVelocity_pct = linearVelocity_pct - rotateVelocity_pct;

		// Compute value to synchronize velocity
		if (!mainUseSimulator) {
			double velocityDifferencePct = LeftMotors.velocity(pct) - RightMotors.velocity(pct);
			double velocityDifferenceInchesPerSecond = (velocityDifferencePct / 100.0) * (600.0 / 60.0) * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);
			double finalVelocityDifferencePct = leftVelocity_pct - rightVelocity_pct;
			double finalVelocityDifferenceInchesPerSecond = (finalVelocityDifferencePct / 100.0) * (600.0 / 60.0) * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);

			// Compute final delta motor velocities
			double velocityDifferenceError = finalVelocityDifferenceInchesPerSecond - velocityDifferenceInchesPerSecond;
			driveAndTurn_synchronizeVelocityPid.computeFromError(velocityDifferenceError);
			double finalDeltaVelocityPct = driveAndTurn_synchronizeVelocityPid.getValue();

			// Update final motor velocities
			leftVelocity_pct += finalDeltaVelocityPct;
			rightVelocity_pct -= finalDeltaVelocityPct;
		}

		// Drive with velocities
		// printf("DisErr: %.3f, AngErr: %.3f\n", distanceError, rotateError);
		botdrive::driveVoltage(aespa_lib::genutil::pctToVolt(leftVelocity_pct), aespa_lib::genutil::pctToVolt(rightVelocity_pct), 10);

		wait(5, msec);
	}

	// Stop
	LeftRightMotors.stop(brake);

	// Correct
	driftCorrector.correct();

	// Settled
	autonfunctions::pid_diff::_driveDistanceError_inches = 0;
	autonfunctions::pid_diff::_isDriveAndTurnSettled = true;
}

}
}
