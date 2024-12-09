#include "Mechanics/botIntake2.h"
#include "Utilities/debugFunctions.h"
#include "Utilities/generalUtility.h"
#include "main.h"

// This mechanic is for two-part intakes: roller + hook

namespace {
	void resolveIntake();
	void resolveIntakeToArm();

	double intakeVelocityPct = 100;

	double toArmHookVelocityPct = 40.0;
	double toArmHookReverseVelocityPct = 70.0;

	/* Derived values (don't change) */

	double intakeVelocityVolt = genutil::pctToVolt(intakeVelocityPct);

	/* Factors */

	double hookFactor = 1.0;
	int hookMode = 0;

	bool autoHookSwitchMode = true;

	int resolveTopState = 0;
	int resolveBottomState = 0;

	bool previousRingDetected = false;
	bool ringDetected = false;

	std::string filterOutColor = "none";
	std::string detectedRingColor;

	bool controlState = true;
}


namespace botintake2 {
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
				if (isStuck && stuckTime.value() > 0.08) {
					resolveTopState = -1;
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

	void setState(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			resolveTopState = state;
			resolveBottomState = state;

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
			resolveTopState = taskState;
			resolveBottomState = taskState;

			return 1;
		});
	}


	void setState2(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			resolveTopState = state;

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
			resolveTopState = taskState;

			return 1;
		});
	}


	void setState3(int state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			resolveBottomState = state;

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
			resolveBottomState = taskState;

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

	void setFilterOutColor(std::string colorText) {
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
		resolveTopState = (resolveTopState > 0) - (resolveTopState < 0);
		resolveBottomState = (resolveBottomState > 0) - (resolveBottomState < 0);

		// Filter out on some detection
		if (previousRingDetected && !ringDetected) {
			if (detectedRingColor == filterOutColor) {
				// Filter out
				wait(40, msec);
				IntakeMotor2.spin(fwd, 0, volt);
				wait(300, msec);
				return;
			}
		}

		// Resolve intake
		switch (resolveBottomState) {
			case 1:
				// Forward
				IntakeMotor1.spin(fwd, intakeVelocityVolt, volt);
				break;
			case -1:
				// Reversed
				IntakeMotor1.spin(fwd, -intakeVelocityVolt, volt);
				break;
			default:
				IntakeMotor1.stop(brakeType::coast);
				break;
		}
		switch (resolveTopState) {
			case 1:
				// Forward
				IntakeMotor2.spin(fwd, intakeVelocityVolt * hookFactor, volt);
				break;
			case -1:
				// Reversed
				IntakeMotor2.spin(fwd, -intakeVelocityVolt * hookFactor, volt);
				break;
			default:
				IntakeMotor2.stop(brakeType::coast);
				break;
		}
	}

	void resolveIntakeToArm() {
		// Reverse hook on some detection
		if (previousRingDetected && !ringDetected) {
			// IntakeMotor1.spin(fwd, 10, pct);

			// Spin hook sequence
			// wait(30, msec);
			IntakeMotor2.spin(fwd, genutil::pctToVolt(-toArmHookReverseVelocityPct), volt);
			wait(300, msec);
			IntakeMotor2.spin(fwd, 0, volt);
			wait(700, msec);

			if (autoHookSwitchMode) {
				hookMode = 0;
			}
		}
		// Otherwise spin hook normally
		else {
			// Spin both
			IntakeMotor1.spin(fwd, intakeVelocityVolt, volt);
			IntakeMotor2.spin(fwd, genutil::pctToVolt(toArmHookVelocityPct), volt);
		}
	}
}
