#include "Mechanics/botIntake.h"
#include "Utilities/debugFunctions.h"
#include "Utilities/generalUtility.h"
#include "main.h"

// This mechanic is for one-part intake with two motors

namespace {
	void resolveIntake();
	void resolveIntakeToArm();

	double intakeVelocityPct = 100;

	double toArmHookVelocityPct = 40.0;
	double toArmHookReverseVelocityPct = 70.0;

	/* Factors */

	double hookFactor = 1.0;
	int hookMode = 0;

	bool autoHookSwitchMode = true;

	int resolveState = 0;

	bool previousRingDetected = false;
	bool ringDetected = false;

	char *filterOutColor = "none";
	char *detectedRingColor;

	bool controlState = true;
}


namespace botintake {
	void runThread() {
		timer stuckTime;
		bool isStuck = false;
		while (true) {
			// Update ring detected
			previousRingDetected = ringDetected;
			double detectedDistance = RingDistanceSensor.objectDistance(distanceUnits::mm);
			if (detectedDistance <= 80.0) {
				ringDetected = true;
			} else {
				ringDetected = false;
			}

			// Update detected ring color
			if (RingOpticalSensor.hue() <= 20 || RingOpticalSensor.hue() >= 340) {
				detectedRingColor = "red";
				// debug::printOnController("Red ring");
			} else if (160 <= RingOpticalSensor.hue() && RingOpticalSensor.hue() <= 240) {
				detectedRingColor = "blue";
				// debug::printOnController("Blue ring");
			}

			// Intake loop
			if (hookMode == 0) {
				// Normal intake
				if (IntakeMotor2.torque() > 0.41) {
					if (!isStuck) {
						stuckTime.clear();
						isStuck = true;
					}
				} else {
					isStuck = false;
				}
				isStuck = false;
				if (isStuck && stuckTime.value() > 0.08) {
					resolveState = -1;
					resolveIntake();
					task::sleep(300);
				} else {
					resolveIntake();
				}
			} else if (hookMode == 1) {
				// Intake to arm
				resolveIntakeToArm();
			}

			task::sleep(5);
		}
	}


	void preauton() {

	}

	void setIntakeVelocity(double velocityPct) {
		intakeVelocityPct = velocityPct;
	}

	double getIntakeVelocity() {
		return intakeVelocityPct;
	}

	void setState(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			resolveState = state;

			return;
		}

		// Set global variables
		_taskState = state;
		_taskDelay = delaySec;

		task setState([]() -> int {
			// Get global variables
			int taskState = _taskState;
			double taskDelay = _taskDelay;

			// Delay setting state
			task::sleep(taskDelay * 1000);

			// Set state here
			resolveState = taskState;

			return 1;
		});
	}

	void switchMode() {
		setHookMode(!hookMode);
	}

	void setHookMode(int mode) {
		hookMode = mode;
		switch (hookMode) {
			case 0:
				debug::printOnController("Intake normal");
				// hookFactor = 1.0;
				break;
			case 1:
				debug::printOnController("Intake to arm");
				// hookFactor = 0.5;
				break;
		}
	}

	void switchFilterColor() {
		if (filterOutColor == "red") {
			filterOutColor = "blue";
			debug::printOnController("filter blue");
		} else {
			filterOutColor = "red";
			debug::printOnController("filter red");
		}
	}

	void setFilterColor(char *colorText) {
		filterOutColor = colorText;
		// debug::printOnController(colorText);
	}

	void control(int state, int hookState) {
		if (canControl()) {
			setState(-state);
			// if (hookState) hookFactor = 0.4;
			// else hookFactor = 1.0;
		}
	}

	bool canControl() {
		return controlState;
	}


	int _taskState;
	double _taskDelay;
}


namespace {
	/// @brief Set the intake to Holding (0) or Released (1). Intake state is modified by setIntakeResolveState(int).
	void resolveIntake() {
		// Make sure intakeResolveState is within [-1, 1]
		resolveState = (resolveState > 0) - (resolveState < 0);

		// Filter out on some detection
		if (previousRingDetected && !ringDetected) {
			if (detectedRingColor == filterOutColor) {
				// Filter out
				wait(45, msec);
				IntakeMotor1.spin(fwd, 0, volt);
				IntakeMotor2.spin(fwd, 0, volt);
				wait(300, msec);
				return;
			}
		}

		// Resolve intake
		switch (resolveState) {
			case 1:
				// Forward
				IntakeMotor1.spin(fwd, genutil::pctToVolt(intakeVelocityPct), volt);
				IntakeMotor2.spin(fwd, genutil::pctToVolt(intakeVelocityPct), volt);
				break;
			case -1:
				// Reversed
				IntakeMotor1.spin(fwd, -genutil::pctToVolt(intakeVelocityPct), volt);
				IntakeMotor2.spin(fwd, -genutil::pctToVolt(intakeVelocityPct), volt);
				break;
			default:
				IntakeMotor1.stop(brakeType::coast);
				IntakeMotor2.stop(brakeType::coast);
				break;
		}
	}

	void resolveIntakeToArm() {
		// Reverse hook on some detection
		if (previousRingDetected && !ringDetected) {
			if (autoHookSwitchMode) {
				hookMode = 0;
			}
		}
		// Otherwise spin hook normally
		else {
			// Spin both
		}
	}
}
