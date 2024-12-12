#include "Autonomous/autonFunctions.h"

#include "AutonUtilities/driftCorrection.h"
#include "AutonUtilities/pidController.h"

#include "Mechanics/botDrive.h"

#include "Utilities/angleUtility.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "Utilities/generalUtility.h"

#include "Simulation/robotSimulator.h"

#include "main.h"

namespace {
	using field::tileLengthIn;

	std::vector<double> getMotorRevolutions();
	double getAverageDifference(std::vector<double> vector1, std::vector<double> vector2);

	bool useRotationSensorForPid = false;
	bool useEncoderForPid = false;

	// DriftCorrection driftCorrector(InertialSensor, -3.276, 3.651);
	DriftCorrection driftCorrector(InertialSensor, 0, 0);

	// Simulator
	bool useSimulator = mainUseSimulator;
}

namespace autonfunctions {
	/// @brief Turn the robot to face a specified angle.
	/// @param rotation The target angle to face in degrees.
	/// @param rotateCenterOffsetIn The offset of the center of rotation.
	/// @param errorRange The allowed degree errors the target angle.
	/// @param runTimeout Maximum seconds the function will run for.
	void turnToAngle(double rotation, double rotateCenterOffsetIn, double errorRange, double runTimeout) {
		turnToAngleVelocity(rotation, 90.0, rotateCenterOffsetIn, errorRange, runTimeout);
	}

	/// @brief Turn the robot to face a specified angle.
	/// @param rotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the rotation.
	/// @param rotateCenterOffsetIn The offset of the center of rotation.
	/// @param errorRange The allowed degree errors the target angle.
	/// @param runTimeout Maximum seconds the function will run for.
	void turnToAngleVelocity(double rotation, double maxVelocityPct, double rotateCenterOffsetIn, double errorRange, double runTimeout) {
		// Set corrector
		driftCorrector.setInitial();

		// Center of rotations
		double leftRotateRadiusIn = botinfo::halfRobotLengthIn + rotateCenterOffsetIn;
		double rightRotateRadiusIn = botinfo::halfRobotLengthIn - rotateCenterOffsetIn;
		double averageRotateRadiusIn = (leftRotateRadiusIn + rightRotateRadiusIn) / 2;

		// Velocity factors
		double leftVelocityFactor = leftRotateRadiusIn / averageRotateRadiusIn;
		double rightVelocityFactor = -rightRotateRadiusIn / averageRotateRadiusIn;

		// Set stopping
		LeftRightMotors.setStopping(brake);

		// PID
		bool useVolt = maxVelocityPct > 25.0;
		// L_vel = L_dist / time
		// R_vel = R_dist / time = L_vel * (R_dist / L_dist)
		// TODO: Tune pid
		PIDController rotateTargetAngleVoltPid(2.5, 0.0, 0.16, errorRange);
		PIDController rotateTargetAngleVelocityPctPid(0.4, 0.0, 0.03, errorRange);

		// Reset timer
		if (useSimulator) {
			robotSimulator.resetTimer();
		}

		timer timeout;
		while (!rotateTargetAngleVoltPid.isSettled() && timeout.value() < runTimeout) {
			// printf("Inertial value: %.3f\n", InertialSensor.rotation(degrees));

			// Get current robot heading
			double currentRotation_degrees = InertialSensor.rotation(degrees);
			if (useSimulator) {
				currentRotation_degrees = angle::swapFieldPolar_degrees(genutil::toDegrees(robotSimulator.angularPosition));
			}

			// Compute heading error
			double rotateError = rotation - currentRotation_degrees;
			if (_useRelativeRotation) {
				rotateError = genutil::modRange(rotateError, 360, -180);
			}

			// Compute heading pid-value from error
			rotateTargetAngleVoltPid.computeFromError(rotateError);
			rotateTargetAngleVelocityPctPid.computeFromError(rotateError);

			// Compute motor rotate velocities
			double averageMotorVelocityPct;
			if (useVolt) {
				averageMotorVelocityPct = rotateTargetAngleVoltPid.getValue();
			} else {
				averageMotorVelocityPct = rotateTargetAngleVelocityPctPid.getValue();
			}
			double leftMotorVelocityPct = leftVelocityFactor * averageMotorVelocityPct;
			double rightMotorVelocityPct = rightVelocityFactor * averageMotorVelocityPct;

			// Scale velocity to maximum
			double scaleFactor = genutil::getScaleFactor(maxVelocityPct, {leftMotorVelocityPct, rightMotorVelocityPct});
			leftMotorVelocityPct *= scaleFactor;
			rightMotorVelocityPct *= scaleFactor;

			// Drive with velocities
			if (!useSimulator) {
				if (useVolt) {
					botdrive::driveVoltage(genutil::pctToVolt(leftMotorVelocityPct), genutil::pctToVolt(rightMotorVelocityPct), 9);
				} else {
					botdrive::driveVelocity(leftMotorVelocityPct, rightMotorVelocityPct);
				}
			} else {
				double leftVelocity = leftMotorVelocityPct / _pathToPctFactor;
				double rightVelocity = rightMotorVelocityPct / _pathToPctFactor;
				double velocity = (leftVelocity + rightVelocity) / 2;
				double angularVelocity = (rightVelocity - leftVelocity) / 2;
				double lookAngle = robotSimulator.angularPosition;
				robotSimulator.velocity = Vector3(velocity * cos(lookAngle), velocity * sin(lookAngle), 0);
				robotSimulator.angularVelocity = angularVelocity;

				robotSimulator.updatePhysics();
				robotSimulator.updateDistance();
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
	/// @param errorRange The allowed tile errors from the target distance.
	/// @param runTimeout Maximum seconds the function will run for.
	void driveDistanceTiles(double distanceTiles, double maxVelocityPct, double errorRange, double runTimeout) {
		driveAndTurnDistanceTiles(distanceTiles, InertialSensor.rotation(), maxVelocityPct, 100.0, errorRange, runTimeout);
	}

	/// @brief Drive the robot for a specified tile distance and rotate it to a specified rotation in degrees.
	/// @param distanceTiles Distance in units of tiles.
	/// @param targetRotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
	/// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
	/// @param errorRange The allowed tile errors from the target distance.
	/// @param runTimeout Maximum seconds the function will run for.
	void driveAndTurnDistanceTiles(double distanceTiles, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
		driveAndTurnDistanceWithInches(distanceTiles * tileLengthIn, targetRotation, maxVelocityPct, maxTurnVelocityPct, errorRange * tileLengthIn, runTimeout);
	}

	/// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
	/// @param distanceInches Distance in units of inches.
	/// @param targetRotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
	/// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
	/// @param errorRange The allowed inch errors from the target distance.
	/// @param runTimeout Maximum seconds the function will run for.
	void driveAndTurnDistanceWithInches(double distanceInches, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
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

		// PID
		// TODO: Tune pid
		PIDController driveTargetDistancePid(12.5, 0, 1.6, errorRange);
		PIDController rotateTargetAnglePid(1.0, 0.05, 0.01, defaultTurnAngleErrorRange);
		PIDController synchronizeVelocityPid(0.4, 0, 0, 5.0);

		// Reset timer
		if (useSimulator) {
			robotSimulator.resetTimer();
		}

		timer timeout;
		while (!(driveTargetDistancePid.isSettled() && rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
			// Compute linear distance error
			double distanceError;
			double targetDistanceInches = distanceInches;
			if (useSimulator) {
				double travelDistance_tiles = (robotSimulator.position - initalSimulatorPosition).getMagnitude() * genutil::signum(targetDistanceInches);
				distanceError = targetDistanceInches - travelDistance_tiles * field::tileLengthIn;
			} else if (useRotationSensorForPid) {
				// printf("Rotation sensor pid\n");
				// Compute current rotation revolutions
				double lookCurrentRevolution = LookRotation.position(rev) - lookRotationInitialRevolution;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = lookCurrentRevolution * (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else if (useEncoderForPid) {
				// Compute current encoder revolutions
				double lookEncoderCurrentRevolution = LookEncoder.rotation(rev) - lookEncoderInitialRevolution;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = lookEncoderCurrentRevolution * (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0);

				// Compute error
				// double revolutionError = (lookEncoderTargetDistanceRevolution - lookEncoderCurrentRevolution);
				// distanceError = revolutionError * (1.0 / trackingLookWheelSensorGearRatio) * (trackingLookWheelCircumIn / 1.0);
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else {
				// Compute average traveled motor revolutions
				std::vector<double> travelRevolutions = getMotorRevolutions();
				double averageTravelRev = getAverageDifference(initRevolutions, travelRevolutions);

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = averageTravelRev * (1.0 / botinfo::driveWheelMotorGearRatio) * (botinfo::driveWheelCircumIn / 1.0);

				// Compute error
				// double revolutionError = (motorTargetDistanceRev - averageTravelRev);
				// distanceError = revolutionError * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			}

			// Compute motor velocity pid-value from error
			driveTargetDistancePid.computeFromError(distanceError);
			double velocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, driveTargetDistancePid.getValue()));

			// Get current robot heading
			double currentRotation_degrees = InertialSensor.rotation(degrees);
			if (useSimulator) {
				currentRotation_degrees = angle::swapFieldPolar_degrees(genutil::toDegrees(robotSimulator.angularPosition));
			}

			// Compute heading error
			double rotateError = targetRotation - currentRotation_degrees;
			if (_useRelativeRotation) {
				rotateError = genutil::modRange(rotateError, 360, -180);
			}

			// Compute heading pid-value from error
			rotateTargetAnglePid.computeFromError(rotateError);
			double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, rotateTargetAnglePid.getValue()));

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
				synchronizeVelocityPid.computeFromError(velocityDifferenceError);
				double finalDeltaVelocityPct = synchronizeVelocityPid.getValue();

				// Update final motor velocities
				leftVelocityPct += finalDeltaVelocityPct;
				rightVelocityPct -= finalDeltaVelocityPct;
			}

			// Drive with velocities
			// printf("DisErr: %.3f, AngErr: %.3f\n", distanceError, rotateError);
			if (!useSimulator) {
				botdrive::driveVoltage(genutil::pctToVolt(leftVelocityPct), genutil::pctToVolt(rightVelocityPct), 10);
			} else {
				double leftVelocity = leftVelocityPct / _pathToPctFactor;
				double rightVelocity = rightVelocityPct / _pathToPctFactor;
				double velocity = (leftVelocity + rightVelocity) / 2;
				double angularVelocity = (rightVelocity - leftVelocity) / 2;
				double lookAngle = robotSimulator.angularPosition;
				robotSimulator.velocity = Vector3(velocity * cos(lookAngle), velocity * sin(lookAngle), 0);
				robotSimulator.angularVelocity = angularVelocity;

				robotSimulator.updatePhysics();
				robotSimulator.updateDistance();
			}

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
