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

#include "Simulation/robotSimulator.h"

#include "main.h"

namespace {
	std::vector<double> getMotorRevolutions();
	double getAverageDifference(std::vector<double> vector1, std::vector<double> vector2);

	bool useRotationSensorForPid = false;
	bool useEncoderForPid = false;

	// DriftCorrection driftCorrector(InertialSensor, -3.276, 3.651);
	DriftCorrection driftCorrector(InertialSensor, 0, 0);

	// Some controllers
	PatienceController angleError_degreesPatience(8, 1.0, false);
	PatienceController driveError_inchesPatience(4, 1.0, false);

	PIDController turnToAngle_rotateTargetAngleVoltPid(2.5, 0.0, 0.16, autonvals::defaultTurnAngleErrorRange);
	PIDController turnToAngle_rotateTargetAngleVelocityPctPid(0.4, 0.0, 0.03, autonvals::defaultTurnAngleErrorRange);

	PIDController driveAndTurn_driveTargetDistancePid(17, 0, 1.6, autonvals::defaultMoveWithInchesErrorRange);
	PIDController driveAndTurn_rotateTargetAnglePid(1.0, 0.05, 0.01, autonvals::defaultTurnAngleErrorRange);
	PIDController driveAndTurn_synchronizeVelocityPid(0.4, 0, 0, 5.0);

	// Simulator
	bool useSimulator = mainUseSimulator;
}

namespace autonfunctions {
	/* PID differential*/

	/// @brief Turn the robot to face a specified angle.
	/// @param rotation The target angle to face in degrees.
	/// @param rotateCenterOffsetIn The offset of the center of rotation.
	/// @param runTimeout Maximum seconds the function will run for.
	void turnToAngle(double rotation, double rotateCenterOffsetIn, double runTimeout) {
		turnToAngleVelocity(rotation, 90.0, rotateCenterOffsetIn, runTimeout);
	}

	/// @brief Turn the robot to face a specified angle.
	/// @param rotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the rotation.
	/// @param rotateCenterOffsetIn The offset of the center of rotation.
	/// @param runTimeout Maximum seconds the function will run for.
	void turnToAngleVelocity(double rotation, double maxVelocityPct, double rotateCenterOffsetIn, double runTimeout) {
		// Set corrector
		driftCorrector.setInitial();

		// Center of rotations
		double leftRotateRadiusIn = botinfo::halfRobotLengthIn + rotateCenterOffsetIn;
		double rightRotateRadiusIn = botinfo::halfRobotLengthIn - rotateCenterOffsetIn;
		double averageRotateRadiusIn = (leftRotateRadiusIn + rightRotateRadiusIn) / 2;

		// Velocity factors
		double leftVelocityFactor = leftRotateRadiusIn / averageRotateRadiusIn;
		double rightVelocityFactor = -rightRotateRadiusIn / averageRotateRadiusIn;
		// L_vel = L_dist / time
		// R_vel = R_dist / time = L_vel * (R_dist / L_dist)

		// Set stopping
		LeftRightMotors.setStopping(brake);

		// Volt config
		bool useVolt = maxVelocityPct > 25.0;

		// Reset PID
		turnToAngle_rotateTargetAngleVoltPid.resetErrorToZero();
		turnToAngle_rotateTargetAngleVelocityPctPid.resetErrorToZero();

		// Reset patience
		angleError_degreesPatience.reset();

		// Reset timer
		timer timeout;

		while (!turnToAngle_rotateTargetAngleVoltPid.isSettled() && timeout.value() < runTimeout) {
			// Check exhausted
			if (angleError_degreesPatience.isExhausted()) {
				break;
			}

			// printf("Inertial value: %.3f\n", InertialSensor.rotation(degrees));

			// Get current robot heading
			double currentRotation_degrees = InertialSensor.rotation(degrees);

			// Compute heading error
			double rotateError = rotation - currentRotation_degrees;
			if (_useRelativeRotation) {
				rotateError = genutil::modRange(rotateError, 360, -180);
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
			double scaleFactor = genutil::getScaleFactor(maxVelocityPct, {leftMotorVelocityPct, rightMotorVelocityPct});
			leftMotorVelocityPct *= scaleFactor;
			rightMotorVelocityPct *= scaleFactor;

			// Drive with velocities
			if (useVolt) {
				botdrive::driveVoltage(genutil::pctToVolt(leftMotorVelocityPct), genutil::pctToVolt(rightMotorVelocityPct), 10);
			} else {
				botdrive::driveVelocity(leftMotorVelocityPct, rightMotorVelocityPct);
			}

			task::sleep(20);
		}

		// Stop
		LeftRightMotors.stop(brake);

		// Correct
		driftCorrector.correct();
	}

	/// @brief Drive straight in the direction of the robot for a specified tile distance.
	/// @param distanceTiles Distance in units of tiles.
	/// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
	/// @param runTimeout Maximum seconds the function will run for.
	void driveDistanceTiles(double distanceTiles, double maxVelocityPct, double runTimeout) {
		driveAndTurnDistanceTiles(distanceTiles, InertialSensor.rotation(), maxVelocityPct, 100.0, runTimeout);
	}

	/// @brief Drive the robot for a specified tile distance and rotate it to a specified rotation in degrees.
	/// @param distanceTiles Distance in units of tiles.
	/// @param targetRotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
	/// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
	/// @param runTimeout Maximum seconds the function will run for.
	void driveAndTurnDistanceTiles(double distanceTiles, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double runTimeout) {
		driveAndTurnDistanceWithInches(distanceTiles * field::tileLengthIn, targetRotation, maxVelocityPct, maxTurnVelocityPct, runTimeout);
	}

	/// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
	/// @param distanceInches Distance in units of inches.
	/// @param targetRotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
	/// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
	/// @param runTimeout Maximum seconds the function will run for.
	void driveAndTurnDistanceWithInches(double distanceInches, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double runTimeout) {
		// Set corrector
		driftCorrector.setInitial();

		// Variables
		// double motorTargetDistanceRev = distanceInches * (1.0 / driveWheelCircumIn) * (driveWheelMotorGearRatio);
		std::vector<double> initRevolutions = getMotorRevolutions();
		// double lookEncoderTargetDistanceRevolution = distanceInches * (1.0 / botinfo::trackingLookWheelCircumIn) * (botinfo::trackingLookWheelEncoderGearRatio);
		double lookEncoderInitialRevolution = LookEncoder.rotation(rev);
		double lookRotationInitialRevolution = LookRotation.position(rev);
		// double rightRotationInitialRevolution = RightRotation.position(rev);
		Vector3 initalSimulatorPosition = robotSimulator.position;

		// Reset PID
		driveAndTurn_driveTargetDistancePid.resetErrorToZero();
		driveAndTurn_rotateTargetAnglePid.resetErrorToZero();
		driveAndTurn_synchronizeVelocityPid.resetErrorToZero();

		// Reset patience
		driveError_inchesPatience.reset();

		// Reset timer
		timer timeout;

		while (!(driveAndTurn_driveTargetDistancePid.isSettled() && driveAndTurn_rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
			// Check exhausted
			if (driveError_inchesPatience.isExhausted()) {
				break;
			}


			/* Linear */

			// Compute linear distance error
			double distanceError;
			double targetDistanceInches = distanceInches;
			if (useSimulator) {
				double travelDistance_tiles = (robotSimulator.position - initalSimulatorPosition).getMagnitude() * genutil::signum(targetDistanceInches);
				distanceError = targetDistanceInches - travelDistance_tiles * field::tileLengthIn;
			} else if (useRotationSensorForPid) {
				// Compute current travel distance in inches
				double lookCurrentRevolution = LookRotation.position(rev) - lookRotationInitialRevolution;
				double currentTravelDistanceInches = lookCurrentRevolution * (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else if (useEncoderForPid) {
				// Compute current travel distance in inches
				double lookEncoderCurrentRevolution = LookEncoder.rotation(rev) - lookEncoderInitialRevolution;
				double currentTravelDistanceInches = lookEncoderCurrentRevolution * (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else {
				// Compute average traveled motor revolutions
				std::vector<double> travelRevolutions = getMotorRevolutions();
				double averageTravelRev = getAverageDifference(initRevolutions, travelRevolutions);

				// Convert revolutions into inches
				double currentTravelDistanceInches = averageTravelRev * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			}

			// Compute motor velocity pid-value from error
			driveAndTurn_driveTargetDistancePid.computeFromError(distanceError);
			double velocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, driveAndTurn_driveTargetDistancePid.getValue()));

			// Update error patience
			driveError_inchesPatience.computePatience(std::fabs(distanceError));


			/* Angular */

			// Get current robot heading
			double currentRotation_degrees = InertialSensor.rotation(degrees);
			if (useSimulator) currentRotation_degrees = angle::swapFieldPolar_degrees(genutil::toDegrees(robotSimulator.angularPosition));

			// Compute heading error
			double rotateError = targetRotation - currentRotation_degrees;
			if (_useRelativeRotation) {
				rotateError = genutil::modRange(rotateError, 360, -180);
			}

			// Compute heading pid-value from error
			driveAndTurn_rotateTargetAnglePid.computeFromError(rotateError);
			double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, driveAndTurn_rotateTargetAnglePid.getValue()));


			/* Combined */

			// Compute final motor velocities
			double leftVelocityPct = velocityPct + rotateVelocityPct;
			double rightVelocityPct = velocityPct - rotateVelocityPct;

			// Compute value to synchronize velocity
			if (!useSimulator) {
				double velocityDifferencePct = LeftMotors.velocity(pct) - RightMotors.velocity(pct);
				double velocityDifferenceInchesPerSecond = (velocityDifferencePct / 100.0) * (600.0 / 60.0) * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);
				double finalVelocityDifferencePct = leftVelocityPct - rightVelocityPct;
				double finalVelocityDifferenceInchesPerSecond = (finalVelocityDifferencePct / 100.0) * (600.0 / 60.0) * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);

				// Compute final delta motor velocities
				double velocityDifferenceError = finalVelocityDifferenceInchesPerSecond - velocityDifferenceInchesPerSecond;
				driveAndTurn_synchronizeVelocityPid.computeFromError(velocityDifferenceError);
				double finalDeltaVelocityPct = driveAndTurn_synchronizeVelocityPid.getValue();

				// Update final motor velocities
				leftVelocityPct += finalDeltaVelocityPct;
				rightVelocityPct -= finalDeltaVelocityPct;
			}

			// Drive with velocities
			// printf("DisErr: %.3f, AngErr: %.3f\n", distanceError, rotateError);
			botdrive::driveVoltage(genutil::pctToVolt(leftVelocityPct), genutil::pctToVolt(rightVelocityPct), 10);

			task::sleep(20);
		}

		// Stop
		LeftRightMotors.stop(coast);

		// Correct
		driftCorrector.correct();
	}


	void setDifferentialUseRelativeRotation(bool useRelativeRotation) {
		_useRelativeRotation = useRelativeRotation;
	}

	bool _useRelativeRotation = false;
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
}
