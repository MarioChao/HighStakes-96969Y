#include "Mechanics/botDrive.h"

#include "Autonomous/autonValues.h"
#include "AutonUtilities/pidController.h"

#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "Utilities/generalUtility.h"
#include "Utilities/fieldInfo.h"

#include "main.h"

namespace {
	using namespace botinfo;
	using botdrive::controlType;

	// Controls
	void controlArcadeTwoStick();
	void controlArcadeSingleStick();
	void drive(double initLeftPct, double initRightPct, double initPolarRotatePct, double rotateCenterOffsetIn = 0);

	// Resolve
	void resolveDriveVelocity();
	void _differentialDriveAtVelocity(double leftVelocity_pct, double rightVelocity_pct);
	void _differentialDriveAtVolt(double leftVelocity_volt, double rightVelocity_volt);

	// Drive mode
	controlType driveMode = controlType::ArcadeTwoStick;
	bool driveModeDebounce = false;

	// Drive config
	double maxDriveVelocityPct = 100.0;
	bool driveUseThread = false;

	// Driving velocity
	double _linearVelocity_pct, _angularVelocity_radPerSecond;

	// Velocity controller
	const double kP = 0.10;
	PIDController driveVelocityLeftMotorPID(kP), driveVelocityRightMotorPID(kP);
}

namespace botdrive {
	void runThread() {
		while (true) {
			if (driveUseThread) {
				resolveDriveVelocity();
			}

			wait(20, msec);
		}
	}

	void preauton() {
		// LeftMotors.setStopping(coast);
		// RightMotors.setStopping(coast);
		LeftMotors.setStopping(brake);
		RightMotors.setStopping(brake);
	}

	/// @brief Switch driving mode between arcade two stick, arcade single stick, and Mario
	void switchDriveMode() {
		if (!driveModeDebounce) {
			driveModeDebounce = true;

			switch (driveMode) {
				case controlType::ArcadeTwoStick:
					driveMode = controlType::ArcadeSingleStick;
					debug::printOnController("Arcade: one stick [X]");
					break;
				case controlType::ArcadeSingleStick:
					driveMode = controlType::ArcadeTwoStick;
					debug::printOnController("Arcade: two stick [X]");
					break;
				default:
					break;
			}
			task::sleep(100);

			driveModeDebounce = false;
		}
	}

	void control() {
		switch (driveMode) {
			case controlType::ArcadeTwoStick:
				controlArcadeTwoStick();
				break;
			case controlType::ArcadeSingleStick:
				controlArcadeSingleStick();
				break;
			default:
				break;
		}
	}

	void setMaxDriveVelocity(double velocityPct) {
		maxDriveVelocityPct = velocityPct;
	}

	double getMaxDriveVelocity() {
		return maxDriveVelocityPct;
	}

	void driveLinegularVelocity(double linearVelocity_pct, double angularVelocity_radPerSecond) {
		_linearVelocity_pct = linearVelocity_pct;
		_angularVelocity_radPerSecond = angularVelocity_radPerSecond;
		resolveDriveVelocity();
	}

	void driveVelocity(double leftVelocityPct, double rightVelocityPct) {
		_linearVelocity_pct = (leftVelocityPct + rightVelocityPct) / 2.0;
		double angularVelocity_tilesPerSecond = (rightVelocityPct - leftVelocityPct) / 2.0 / autonvals::tilesPerSecond_to_pct;
		_angularVelocity_radPerSecond = angularVelocity_tilesPerSecond / (botinfo::halfRobotLengthIn * (1.0 / field::tileLengthIn));
		resolveDriveVelocity();
	}

	void driveVoltage(double leftVoltageVolt, double rightVoltageVolt, double clampMaxVoltage) {
		// Preprocess config
		double maxVoltage = 12.0;
		clampMaxVoltage = fabs(clampMaxVoltage);

		// Scale voltages if overshoot
		double scaleFactor = genutil::getScaleFactor(maxVoltage, {leftVoltageVolt, rightVoltageVolt});
		leftVoltageVolt *= scaleFactor;
		rightVoltageVolt *= scaleFactor;

		// Clamp
		leftVoltageVolt = genutil::clamp(leftVoltageVolt, -clampMaxVoltage, clampMaxVoltage);
		rightVoltageVolt = genutil::clamp(rightVoltageVolt, -clampMaxVoltage, clampMaxVoltage);

		// Drive
		_differentialDriveAtVolt(leftVoltageVolt, rightVoltageVolt);
	}
}

namespace {
	/// @brief Drive in arcade mode (Axis3 forward/backward, Axis1 rotation)
	void controlArcadeTwoStick() {
		double axis3 = Controller1.Axis3.position();
		if (fabs(axis3) < 2) axis3 = 0;

		double axis1 = Controller1.Axis1.position();
		if (fabs(axis1) < 2) axis1 = 0;

		drive(axis3, axis3, -axis1 * 0.8);
	}

	/// @brief Drive in arcade mode (Axis3 forward/backward, Axis4 rotation)
	void controlArcadeSingleStick() {
		double axis3 = Controller1.Axis3.position();
		if (fabs(axis3) < 2) axis3 = 0;

		double axis4 = Controller1.Axis4.position();
		if (fabs(axis4) < 2) axis4 = 0;

		drive(axis3, axis3, -axis4);
	}

	void drive(double initLeftPct, double initRightPct, double initPolarRotatePct, double rotateCenterOffsetIn) {
		// Compute scaled rotations
		double leftRotateRadiusIn = halfRobotLengthIn + rotateCenterOffsetIn;
		double rightRotateRadiusIn = halfRobotLengthIn - rotateCenterOffsetIn;
		double leftPolarRotatePct = initPolarRotatePct * (leftRotateRadiusIn / halfRobotLengthIn);
		double rightPolarRotatePct = initPolarRotatePct * (rightRotateRadiusIn / halfRobotLengthIn);

		// Compute final percentages
		double leftPct = initLeftPct - leftPolarRotatePct;
		double rightPct = initRightPct + rightPolarRotatePct;

		if (true) {
			// Drive
			// botdrive::driveVelocity(leftPct, rightPct);
			botdrive::driveVoltage(genutil::pctToVolt(leftPct), genutil::pctToVolt(rightPct), 12);
		} else {
			// Scale percentages if overshoot
			double scaleFactor = genutil::getScaleFactor(maxDriveVelocityPct, {leftPct, rightPct});
			leftPct *= scaleFactor;
			rightPct *= scaleFactor;

			// Spin motors at volt
			// LeftMotors.spin(fwd, leftPct, pct);
			// RightMotors.spin(fwd, rightPct, pct);
			if (fabs(leftPct) < 5) {
				LeftMotors.stop();
			} else {
				LeftMotors.spin(fwd, genutil::pctToVolt(leftPct), volt);
			}
			if (fabs(rightPct) < 5) {
				RightMotors.stop();
			} else {
				RightMotors.spin(fwd, genutil::pctToVolt(rightPct), volt);
			}
		}
	}

	void resolveDriveVelocity() {
		// Differential drive

		// Convert angular velocity to wheel's linear velocity
		double rotationLinearVelocity_tilesPerSecond = _angularVelocity_radPerSecond * botinfo::halfRobotLengthIn * (1.0 / field::tileLengthIn);
		double rotationLinearVelocity_pct = rotationLinearVelocity_tilesPerSecond * autonvals::tilesPerSecond_to_pct;

		// Compute differential velocity
		double leftVelocity_pct = _linearVelocity_pct - rotationLinearVelocity_pct;
		double rightVelocity_pct = _linearVelocity_pct + rotationLinearVelocity_pct;

		// Drive at velocity
		_differentialDriveAtVelocity(leftVelocity_pct, rightVelocity_pct);
	}

	void _differentialDriveAtVelocity(double leftVelocity_pct, double rightVelocity_pct) {
		// Scale percentages if overshoot
		double scaleFactor = genutil::getScaleFactor(100.0, {leftVelocity_pct, rightVelocity_pct});
		leftVelocity_pct *= scaleFactor;
		rightVelocity_pct *= scaleFactor;

		if (false) {
			// Calculate velocity errors
			double leftVelocity_error = leftVelocity_pct - LeftMotors.velocity(pct);
			double rightVelocity_error = rightVelocity_pct - RightMotors.velocity(pct);

			// Compute needed voltage to maintain velocity
			driveVelocityLeftMotorPID.computeFromError(leftVelocity_error);
			driveVelocityRightMotorPID.computeFromError(rightVelocity_error);
			double leftDeltaVolt = driveVelocityLeftMotorPID.getValue();
			double rightDeltaVolt = driveVelocityRightMotorPID.getValue();

			// Compute final voltage
			double leftVelocity_volt = LeftMotors.voltage(volt) + leftDeltaVolt;
			double rightVelocity_volt = RightMotors.voltage(volt) + rightDeltaVolt;

			// Drive at volt
			botdrive::driveVoltage(leftVelocity_volt, rightVelocity_volt, 11);
			printf("Err: %.3f, %.3f, Lvolt: %.3f, Rvolt: %.3f\n", leftVelocity_error, rightVelocity_error, leftVelocity_volt, rightVelocity_volt);
		} else {
			// Spin motors
			LeftMotors.spin(fwd, leftVelocity_pct, pct);
			RightMotors.spin(fwd, rightVelocity_pct, pct);
		}
	}

	void _differentialDriveAtVolt(double leftVelocity_volt, double rightVelocity_volt) {
		// Scale voltages if overshoot
		double scaleFactor = genutil::getScaleFactor(12, {leftVelocity_volt, rightVelocity_volt});
		leftVelocity_volt *= scaleFactor;
		rightVelocity_volt *= scaleFactor;

		// Spin
		LeftMotors.spin(fwd, leftVelocity_volt, volt);
		RightMotors.spin(fwd, rightVelocity_volt, volt);
	}
}
