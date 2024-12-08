#include "Mechanics/botDrive.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "Utilities/generalUtility.h"
#include "main.h"

namespace {
	using namespace botinfo;
	using botdrive::controlType;

	void controlArcadeTwoStick();
	void controlArcadeSingleStick();
	void drive(double initLeftPct, double initRightPct, double initPolarRotatePct, double rotateCenterOffsetIn = 0);

	// Drive mode
	controlType driveMode = controlType::ArcadeTwoStick;
	bool driveModeDebounce = false;

	// Drive config
	double maxDriveVelocityPct = 100.0;
}

namespace botdrive {
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

	void driveVelocity(double leftVelocityPct, double rightVelocityPct) {
		// Scale percentages if overshoot
		double scaleFactor = genutil::getScaleFactor(100.0, {leftVelocityPct, rightVelocityPct});
		leftVelocityPct *= scaleFactor;
		rightVelocityPct *= scaleFactor;

		// Spin motors
		LeftMotors.spin(fwd, leftVelocityPct, pct);
		RightMotors.spin(fwd, rightVelocityPct, pct);
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

		// Spin motors
		LeftMotors.spin(fwd, leftVoltageVolt, volt);
		RightMotors.spin(fwd, rightVoltageVolt, volt);
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

		// Scale percentages if overshoot
		double scaleFactor = genutil::getScaleFactor(maxDriveVelocityPct, {leftPct, rightPct});
		leftPct *= scaleFactor;
		rightPct *= scaleFactor;

		if (true) {
			// Drive
			botdrive::driveVoltage(genutil::pctToVolt(leftPct), genutil::pctToVolt(rightPct), 12);
		} else {
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
}
