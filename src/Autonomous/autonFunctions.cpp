#include "Autonomous/autonFunctions.h"

#include "AutonUtilities/driftCorrection.h"
#include "AutonUtilities/pidControl.h"
#include "AutonUtilities/motionProfile.h"

#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botArmPneumatics.h"
#include "Mechanics/goalClamp.h"
#include "Utilities/angleFunctions.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "Utilities/generalUtility.h"
#include "main.h"

namespace {
	using namespace angle;
	using namespace botinfo;
	using namespace field;
	using std::vector;

	vector<double> getMotorRevolutions();
	double getAverageDifference(vector<double> vector1, vector<double> vector2);
	void driveVelocity(double leftVelocityPct, double rightVelocityPct);
	void driveVoltage(double leftVoltagePct, double rightVoltagePct, double clampMaxVoltage);

	double setFrontWings_DelaySec;
	double setLeftWing_DelaySec;
	double setRightWing_DelaySec;
	double setWings_DelaySec;
	bool setFrontWings_WingState;
	bool setLeftWing_LeftWingState;
	bool setRightWing_RightWingState;
	bool setWings_WingsState;

	bool useRotationSensorForPid = false;
	bool useEncoderForPid = false;

	DriftCorrection driftCorrector(InertialSensor, -3.276, 3.651);
	// DriftCorrection driftCorrector(InertialSensor, 0, 0);
}

namespace autonfunctions {
	/// @brief Set the inertial sensor's absolute angle reading to a specified value. Doesn't turn the robot.
	/// @param rotation The angle (in degrees) to be set for the current orientation.
	void setRotation(double rotation) {
		InertialSensor.setRotation(rotation, deg);
	}

	/// @brief Turn the robot to face a specified angle.
	/// @param rotation The target angle to face in degrees.
	/// @param rotateCenterOffsetIn The offset of the center of rotation.
	/// @param errorRange The allowed degree errors the target angle.
	/// @param runTimeout Maximum seconds the function will run for.
	void turnToAngle(double rotation, double rotateCenterOffsetIn, double errorRange, double runTimeout) {
		turnToAngleVelocity(rotation, 100.0, rotateCenterOffsetIn, errorRange, runTimeout);
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
		double leftRotateRadiusIn = halfRobotLengthIn + rotateCenterOffsetIn;
		double rightRotateRadiusIn = halfRobotLengthIn - rotateCenterOffsetIn;
		double averageRotateRadiusIn = (leftRotateRadiusIn + rightRotateRadiusIn) / 2;

		// Velocity factors
		double leftVelocityFactor = leftRotateRadiusIn / averageRotateRadiusIn;
		double rightVelocityFactor = -rightRotateRadiusIn / averageRotateRadiusIn;

		// Set stopping
		LeftRightMotors.setStopping(brake);

		// PID
		// L_vel = L_dist / time
		// R_vel = R_dist / time = L_vel * (R_dist / L_dist)
		// TODO: Tune pid
		PIDControl rotateTargetAnglePid(1.8, 0.002, 1.7, errorRange);
		timer timeout;
		while (!rotateTargetAnglePid.isSettled() && timeout.value() < runTimeout) {
			// printf("Inertial value: %.3f\n", InertialSensor.rotation(degrees));
			// Compute rotate error
			double rotateError = rotation - InertialSensor.rotation();
			rotateTargetAnglePid.computeFromError(rotateError);

			// Compute motor rotate velocities
			double averageMotorVelocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, rotateTargetAnglePid.getValue()));
			double leftMotorVelocityPct = leftVelocityFactor * averageMotorVelocityPct;
			double rightMotorVelocityPct = rightVelocityFactor * averageMotorVelocityPct;

			// Drive with velocities
			driveVoltage(genutil::pctToVolt(leftMotorVelocityPct), genutil::pctToVolt(rightMotorVelocityPct), 7);

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
	void driveAndTurnDistanceTilesMotionProfile(double distanceTiles, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
		driveAndTurnDistanceWithInchesMotionProfile(distanceTiles * tileLengthIn, targetRotation, maxVelocityPct, maxTurnVelocityPct, errorRange * tileLengthIn, runTimeout);
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
		// Test
		// driveAndTurnDistanceInchesMotionProfile(distanceInches, targetRotation, maxVelocityPct, maxTurnVelocityPct, errorRange, runTimeout);
		// return;

		// Set corrector
		driftCorrector.setInitial();

		// Variables
		// double motorTargetDistanceRev = distanceInches * (1.0 / driveWheelCircumIn) * (driveWheelMotorGearRatio);
		vector<double> initRevolutions = getMotorRevolutions();
		// double lookEncoderTargetDistanceRevolution = distanceInches * (1.0 / trackingLookWheelCircumIn) * (trackingLookWheelEncoderGearRatio);
		double lookEncoderInitialRevolution = LookEncoder.rotation(rev);
		double lookRotationInitialRevolution = LookRotation.position(rev);
		double rightRotationInitialRevolution = RightRotation.position(rev);

		// PID
		// TODO: Tune pid
		PIDControl driveTargetDistancePid(12.5, 0, 80, errorRange);
		PIDControl rotateTargetAnglePid(1.0, 0.001, 0.5, defaultTurnAngleErrorRange);
		PIDControl synchronizeVelocityPid(0.4, 0, 0, 5.0);

		timer timeout;
		printf("Drive with inches\n");
		while (!(driveTargetDistancePid.isSettled() && rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
			// Compute linear distance error
			double distanceError;
			double targetDistanceInches = distanceInches;
			if (useRotationSensorForPid) {
				printf("Rotation sensor pid\n");
				// Compute current rotation revolutions
				double lookCurrentRevolution = LookRotation.position(rev) - lookRotationInitialRevolution;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = lookCurrentRevolution * (1.0 / trackingLookWheelSensorGearRatio) * (trackingLookWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else if (useEncoderForPid) {
				// Compute current encoder revolutions
				double lookEncoderCurrentRevolution = LookEncoder.rotation(rev) - lookEncoderInitialRevolution;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = lookEncoderCurrentRevolution * (1.0 / trackingLookWheelSensorGearRatio) * (trackingLookWheelCircumIn / 1.0);

				// Compute error
				// double revolutionError = (lookEncoderTargetDistanceRevolution - lookEncoderCurrentRevolution);
				// distanceError = revolutionError * (1.0 / trackingLookWheelSensorGearRatio) * (trackingLookWheelCircumIn / 1.0);
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else {
				// Compute average traveled motor revolutions
				vector<double> travelRevolutions = getMotorRevolutions();
				double averageTravelRev = getAverageDifference(initRevolutions, travelRevolutions);

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = averageTravelRev * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);

				// Compute error
				// double revolutionError = (motorTargetDistanceRev - averageTravelRev);
				// distanceError = revolutionError * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			}

			// Compute motor velocity pid-value from error
			driveTargetDistancePid.computeFromError(distanceError);
			double velocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, driveTargetDistancePid.getValue()));

			// Compute heading pid-value from error
			double rotateError = (targetRotation - InertialSensor.rotation());
			rotateTargetAnglePid.computeFromError(rotateError);
			double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, rotateTargetAnglePid.getValue()));

			// Compute final motor velocities
			double leftVelocityPct = velocityPct + rotateVelocityPct;
			double rightVelocityPct = velocityPct - rotateVelocityPct;

			// Compute value to synchronize velocity
			double velocityDifferencePct = LeftMotors.velocity(pct) - RightMotors.velocity(pct);
			double velocityDifferenceInchesPerSecond = (velocityDifferencePct / 100.0) * (600.0 / 60.0) * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);
			double finalVelocityDifferencePct = leftVelocityPct - rightVelocityPct;
			double finalVelocityDifferenceInchesPerSecond = (finalVelocityDifferencePct / 100.0) * (600.0 / 60.0) * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);

			// Compute final delta motor velocities
			double velocityDifferenceError = finalVelocityDifferenceInchesPerSecond - velocityDifferenceInchesPerSecond;
			synchronizeVelocityPid.computeFromError(velocityDifferenceError);
			double finalDeltaVelocityPct = synchronizeVelocityPid.getValue();

			// Update final motor velocities
			leftVelocityPct += finalDeltaVelocityPct;
			rightVelocityPct -= finalDeltaVelocityPct;

			// Drive with velocities
			driveVoltage(genutil::pctToVolt(leftVelocityPct), genutil::pctToVolt(rightVelocityPct), 10);
			// printf("DisErr: %.3f, AngErr: %.3f\n", distanceError, rotateError);

			task::sleep(20);
		}

		// Stop
		LeftRightMotors.stop(coast);

		// Correct
		driftCorrector.correct();
	}

	/// @brief Drive the robot for a specified distance in inches and rotate it to a specified rotation in degrees.
	/// @param distanceInches Distance in units of inches.
	/// @param targetRotation The target angle to face in degrees.
	/// @param maxVelocityPct Maximum velocity of the drive. (can > 100)
	/// @param maxTurnVelocityPct Maximum rotational velocity of the drive. (can > 100)
	/// @param errorRange The allowed inch errors from the target distance.
	/// @param runTimeout Maximum seconds the function will run for.
	void driveAndTurnDistanceWithInchesMotionProfile(double distanceInches, double targetRotation, double maxVelocityPct, double maxTurnVelocityPct, double errorRange, double runTimeout) {
		// Variables
		double leftMotorInitRev = LeftMotors.position(rev);
		double rightMotorInitRev = RightMotors.position(rev);
		double lookEncoderInitialRevolution = LookEncoder.rotation(rev);
		double lookRotationInitialRevolution = LookRotation.position(rev);

		// PID
		// TODO: Tune pid
		PIDControl driveTargetDistancePid(5.2, 0, 0.1, errorRange);
		PIDControl rotateTargetAnglePid(0.6, 0, 0, defaultTurnAngleErrorRange);

		// Motion profile
		MotionProfile driveSpeedMotionProfile;
		double maxVelocityInchesPerSecond = (maxVelocityPct / 100.0) * (600.0 / 60.0) * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);
		driveSpeedMotionProfile.setModeAcceleration(maxVelocityInchesPerSecond * 1.30, maxVelocityInchesPerSecond * 1.30, maxVelocityInchesPerSecond);
		// driveSpeedMotionProfile.createProfile(motorTargetDistanceRev * driveMotorRevToPercentSecFactor);
		driveSpeedMotionProfile.createProfile(distanceInches);
		driveSpeedMotionProfile.start();
		printf(" --- Motion running for %.3f seconds. ---\n", driveSpeedMotionProfile.getMotionEndTime());

		// TODO: modify this function to make it fast and precise
		timer timeout;
		while (!(driveSpeedMotionProfile.isDone() && driveTargetDistancePid.isSettled() && rotateTargetAnglePid.isSettled()) && timeout.value() < runTimeout) {
			// Compute linear distance error
			double distanceError;
			double targetDistanceInches = driveSpeedMotionProfile.getNextSectionDistance();
			if (useRotationSensorForPid) {
				// Compute current revolutions
				double lookCurrentRevolution = LookRotation.position(rev) - lookRotationInitialRevolution;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = lookCurrentRevolution * (1.0 / trackingLookWheelSensorGearRatio) * (trackingLookWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else if (useEncoderForPid) {
				// Compute current encoder revolutions
				double lookEncoderCurrentRevolution = LookEncoder.rotation(rev) - lookEncoderInitialRevolution;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = lookEncoderCurrentRevolution * (1.0 / trackingLookWheelSensorGearRatio) * (trackingLookWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			} else {
				// Compute average traveled motor revolutions
				double leftTravelRev = LeftMotors.position(rev) - leftMotorInitRev;
				double rightTravelRev = RightMotors.position(rev) - rightMotorInitRev;
				double averageTravelRev = (leftTravelRev + rightTravelRev) / 2;

				// Convert current revolutions into distance inches
				double currentTravelDistanceInches = averageTravelRev * (1.0 / driveWheelMotorGearRatio) * (driveWheelCircumIn / 1.0);

				// Compute error
				distanceError = targetDistanceInches - currentTravelDistanceInches;
			}

			// Compute target velocity from motion profile
			double targetVelocityInchesPerSecond = driveSpeedMotionProfile.getVelocity();
			double targetVelocityPct = targetVelocityInchesPerSecond * (1.0 / driveWheelCircumIn) * (driveWheelMotorGearRatio / 1.0) * (60.0 / 600.0) * 100.0;

			// Compute additional velocity pid-value from distance error
			driveTargetDistancePid.computeFromError(distanceError);
			double velocityFromDistancePid = driveTargetDistancePid.getValue();

			// Compute motor velocity from pid-values
			double linearVelocityPct = targetVelocityPct + (velocityFromDistancePid - targetVelocityPct) * fabs(distanceError / 10.0);
			linearVelocityPct = fmin(maxVelocityPct, fmax(-maxVelocityPct, linearVelocityPct));
			printf("ProfVel%%: %.3f, LinVel%%: %.3f, DisErr: %.3f, DisPid: %.3f\n", targetVelocityPct, linearVelocityPct, distanceError, velocityFromDistancePid);

			// Compute heading pid-value from error
			double rotateError = (targetRotation - InertialSensor.rotation());
			rotateTargetAnglePid.computeFromError(rotateError);
			double rotateVelocityPct = fmin(maxTurnVelocityPct, fmax(-maxTurnVelocityPct, rotateTargetAnglePid.getValue()));

			// Compute final motor velocities
			double leftVelocityPct = linearVelocityPct + rotateVelocityPct;
			double rightVelocityPct = linearVelocityPct - rotateVelocityPct;

			// Drive with voltage
			driveVoltage(genutil::pctToVolt(leftVelocityPct), genutil::pctToVolt(rightVelocityPct), 10);

			task::sleep(20);
		}

		// Stop
		LeftRightMotors.stop(brake);
	}

	/// @brief Set the state of the intake.
	/// @param state Forward: 1, released: 0, reversed: -1
	/// @param delaySec Number of seconds to wait before setting the state (in a task).
	void setIntakeState(int state, double delaySec) {
		if (intakePart == 1) botintake::setState(state, delaySec);
		else botintake2::setState(state, delaySec);
	}


	/// @brief Set the state of the top intake.
	/// @param state Forward: 1, released: 0, reversed: -1
	/// @param delaySec Number of seconds to wait before setting the state (in a task).
	void setIntakeTopState(int state, double delaySec) {
		if (intakePart == 1) return;
		else botintake2::setState2(state, delaySec);
	}


	/// @brief Set the state of the bottom intake.
	/// @param state Forward: 1, released: 0, reversed: -1
	/// @param delaySec Number of seconds to wait before setting the state (in a task).
	void setIntakeBottomState(int state, double delaySec) {
		if (intakePart == 1) return;
		else botintake2::setState3(state, delaySec);
	}

	/// @brief Set the hook mode of the intake.
	/// @param state Normal: 0, to arm: 1
	void setIntakeHookMode(int state) {
		if (intakePart == 1) return;
		else botintake2::setHookMode(state);
	}

	void setArmHangState(int state, double delaySec) {
		botarmpneu::setState(state, delaySec);
	}

	/// @brief Set the state of Left Wing's pneumatic.
   /// @param state Expanded: true, retracted: false.
   /// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setGoalClampState(bool state, double delaySec) {
		goalclamp::setState(state, delaySec);
	}

	/// @brief Set the state of Front Wings's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setFrontWingsState(bool state, double delaySec) {
		setFrontWings_WingState = state;
		setFrontWings_DelaySec = delaySec;
		task setPneumaticState([]() -> int {
			int taskState = setFrontWings_WingState;

			if (setFrontWings_DelaySec > 1e-9) {
				task::sleep(setFrontWings_DelaySec * 1000);
			}
			FrontWingsPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of Left Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setLeftWingState(bool state, double delaySec) {
		setLeftWing_LeftWingState = state;
		setLeftWing_DelaySec = delaySec;
		task setPneumaticState([]() -> int {
			int taskState = setLeftWing_LeftWingState;

			if (setLeftWing_DelaySec > 1e-9) {
				task::sleep(setLeftWing_DelaySec * 1000);
			}
			LeftWingPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of Right Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setRightWingState(bool state, double delaySec) {
		setRightWing_RightWingState = state;
		setRightWing_DelaySec = delaySec;
		task setPneumaticState([]() -> int {
			int taskState = setRightWing_RightWingState;

			if (setRightWing_DelaySec > 1e-9) {
				task::sleep(setRightWing_DelaySec * 1000);
			}
			RightWingPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of Left and Right Wing's pneumatic.
	/// @param state Expanded: true, retracted: false.
	/// @param delaySec Number of seconds to wait before setting the pneumatic state (in a task).
	void setBackWingsState(bool state, double delaySec) {
		setWings_WingsState = state;
		setWings_DelaySec = delaySec;
		task setPneumaticsState([]() -> int {
			int taskState = setWings_WingsState;

			if (setWings_DelaySec > 1e-9) {
				task::sleep(setWings_DelaySec * 1000);
			}
			LeftWingPneumatic.set(taskState);
			RightWingPneumatic.set(taskState);
			return 1;
		});
	}

	/// @brief Set the state of the lift's pneumatic.
	/// @param state Lifted: true, lowered: false
	void setIntakeLiftState(bool state) {
		IntakeLiftPneumatic.set(state);
	}
}

namespace {
	/* Can remove these two functions*/
	vector<double> getMotorRevolutions() {
		vector<double> ret = {
			LeftMotorB.position(rev),
			LeftMotorC.position(rev),
			RightMotorB.position(rev),
			RightMotorC.position(rev),
		};
		return ret;
	}
	/// @brief Returns the average value of vector2[i] - vector1[i].
	double getAverageDifference(vector<double> vector1, vector<double> vector2) {
		int vectorSize = std::min((int) vector1.size(), (int) vector2.size());
		double totalDifference = 0;
		for (int i = 0; i < vectorSize; i++) {
			double difference = vector2[i] - vector1[i];
			totalDifference += difference;
		}
		double averageDifference = totalDifference / vectorSize;
		return averageDifference;
	}

	void driveVelocity(double leftVelocityPct, double rightVelocityPct) {
		// Scale percentages if overshoot
		double scaleFactor = 100.0 / fmax(100.0, fmax(fabs(leftVelocityPct), fabs(rightVelocityPct)));
		leftVelocityPct *= scaleFactor;
		rightVelocityPct *= scaleFactor;

		// Spin motors
		LeftMotors.spin(fwd, leftVelocityPct, pct);
		RightMotors.spin(fwd, rightVelocityPct, pct);
	}
	void driveVoltage(double leftVoltageVolt, double rightVoltageVolt, double clampMaxVoltage) {
		double maxVoltage = 12.0;

		// Scale voltages if overshoot
		double scaleFactor = maxVoltage / fmax(maxVoltage, fmax(fabs(leftVoltageVolt), fabs(rightVoltageVolt)));
		leftVoltageVolt *= scaleFactor;
		rightVoltageVolt *= scaleFactor;

		// Clamp
		leftVoltageVolt = genutil::clamp(leftVoltageVolt, -clampMaxVoltage, clampMaxVoltage);
		rightVoltageVolt = genutil::clamp(rightVoltageVolt, -clampMaxVoltage, clampMaxVoltage);

		// Spin motors
		LeftMotors.spin(fwd, leftVoltageVolt, volt);
		RightMotors.spin(fwd, rightVoltageVolt, volt);
	}
}
