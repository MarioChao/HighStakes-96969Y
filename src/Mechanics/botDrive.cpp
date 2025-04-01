#include "Mechanics/botDrive.h"

#include "Autonomous/autonValues.h"
#include "Pas1-Lib/Auton/Control-Loops/pid.h"
#include "Pas1-Lib/Auton/Control-Loops/feedforward.h"

#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Utilities/debugFunctions.h"
#include "Utilities/fieldInfo.h"

#include "global-vars.h"
#include "main.h"

namespace {
	using botdrive::controlType;
	using pas1_lib::auton::control_loops::ForwardController;
	using pas1_lib::auton::control_loops::PIDController;

	// Controls
	void controlArcadeTwoStick();
	void controlArcadeSingleStick();
	void drive(double left_pct, double right_pct, double angular_pct);

	// Drive mode
	controlType driveMode = controlType::ArcadeTwoStick;
	bool driveModeDebounce = false;

	// Drive config
	double maxDriveVelocityPct = 100.0;

	// Control state
	bool controlState = true;
}

namespace botdrive {
	void runThread() {
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
		if (!canControl()) {
			return;
		}
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

	bool canControl() {
		return controlState;
	}

	void setControlState(bool canControl) {
		controlState = canControl;
	}

	void setMaxDriveVelocity(double velocityPct) {
		maxDriveVelocityPct = velocityPct;
	}

	double getMaxDriveVelocity() {
		return maxDriveVelocityPct;
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

	void drive(double left_pct, double right_pct, double angular_pct) {
		// Limit scale
		left_pct -= angular_pct;
		right_pct += angular_pct;
		double scaleFactor = aespa_lib::genutil::getScaleFactor(maxDriveVelocityPct, {left_pct, right_pct});
		left_pct *= scaleFactor;
		right_pct *= scaleFactor;

		// Get linear and angular
		double linear_pct = (left_pct + right_pct) / 2;
		angular_pct = (right_pct - left_pct) / 2;

		// Drive
		robotChassis.control_local2d(0, linear_pct, angular_pct);
	}
}
