#include "Mechanics/botIntake.h"

#include "Mechanics/redirect.h"
#include "Utilities/debugFunctions.h"
#include "Utilities/generalUtility.h"
#include "main.h"

// This mechanic is for one-part intake with two motors

namespace {
	void resolveIntake();

	double intakeVelocityPct = 100;

	/* Factors */

	bool colorFilterEnabled = true;
	const bool disableColorFilter = false;

	int resolveState = 0;

	bool previousRingDetected = false;
	bool ringDetected = false;

	std::string filterOutColor = "none";
	std::string detectedRingColor;
	bool isDetectingRing;

	bool isStoringRing = false;

	// Reverse intake loop
	const int reverseIntakeThreshold = 100;
	const int reverseIntakeFor = 10;
	int reverseIntakeCalls = 0;

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

			// Update detecting ring
			isDetectingRing = RingOpticalSensor.isNearObject();
			if (isDetectingRing) {
				// Update detected ring color
				if (RingOpticalSensor.hue() <= 20 || RingOpticalSensor.hue() >= 340) {
					detectedRingColor = "red";
					// debug::printOnController("Red ring");
				} else if (180 <= RingOpticalSensor.hue() && RingOpticalSensor.hue() <= 230) {
					detectedRingColor = "blue";
					// debug::printOnController("Blue ring");
				} else {
					detectedRingColor = "none";
					// debug::printOnController("No ring");
				}
			}

			/* Intake loop */

			if (!isStoringRing) {
				/* Normal intake */

				// Detect stuck
				if (IntakeMotor2.torque() > 0.41) {
					if (!isStuck) {
						stuckTime.clear();
						isStuck = true;
					}
				} else {
					isStuck = false;
				}
				// Override stuck
				isStuck = false;

				// Check reverse intake
				bool reverseIntake = false;
				reverseIntakeCalls++;
				if (reverseIntakeCalls > reverseIntakeThreshold) {
					reverseIntake = true;
					if (reverseIntakeCalls - reverseIntakeThreshold > reverseIntakeFor) {
						reverseIntakeCalls = 0;
					}
				}
				// Override reverse
				reverseIntake = false;

				if (isStuck && stuckTime.value() > 0.08) {
					// Reverse on stuck
					resolveState = -1;
					resolveIntake();
					task::sleep(300);
				} else if (reverseIntake) {
					resolveState = -1;
					resolveIntake();
				} else {
					resolveIntake();
				}
			} else {
				/* Store ring */

				resolveState = 1;
				resolveIntake();

				if (isDetectingRing && detectedRingColor != "none") {
					if (detectedRingColor != filterOutColor || !colorFilterEnabled) {
						wait(10, msec);
						resolveState = 0;
						isStoringRing = false;
					}
				}
			}

			wait(5, msec);
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

	bool isColorFiltering() {
		return colorFilterEnabled;
	}

	void setColorFiltering(bool state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			colorFilterEnabled = state;

			return;
		}

		// Set global variables
		_colorFilterTaskState = state;
		_colorFilterTaskDelay = delaySec;

		task setState([]() -> int {
			// Get global variables
			int taskState = _colorFilterTaskState;
			double taskDelay = _colorFilterTaskDelay;

			// Delay setting state
			task::sleep(taskDelay * 1000);

			// Set state here
			colorFilterEnabled = taskState;

			return 1;
		});
	}

	void switchFilterColor() {
		if (filterOutColor == "red") {
			filterOutColor = "blue";
			debug::printOnController("filter out blue");
		} else {
			filterOutColor = "red";
			debug::printOnController("filter out red");
		}
	}

	void setFilterOutColor(std::string colorText) {
		filterOutColor = colorText;
		// debug::printOnController(colorText);
	}

	void setIntakeStoreRing(bool state, double delaySec) {
		// Check for instant set
		if (delaySec <= 1e-9) {
			// Set state here
			isStoringRing = state;

			return;
		}

		// Set global variables
		_storeRingTaskState = state;
		_storeRingTaskDelay = delaySec;

		task setState([]() -> int {
			// Get global variables
			int taskState = _storeRingTaskState;
			double taskDelay = _storeRingTaskDelay;

			// Delay setting state
			task::sleep(taskDelay * 1000);

			// Set state here
			isStoringRing = taskState;

			return 1;
		});
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

	bool _colorFilterTaskState;
	double _colorFilterTaskDelay;

	bool _storeRingTaskState;
	double _storeRingTaskDelay;
}


namespace {
	/// @brief Set the intake to Holding (0) or Released (1). Intake state is modified by setIntakeResolveState(int).
	void resolveIntake() {
		// Make sure intakeResolveState is within [-1, 1]
		resolveState = (resolveState > 0) - (resolveState < 0);

		// Filter out on some detection
		if (colorFilterEnabled) {
			if (disableColorFilter) {
				if (redirect::getState() == 1) {
					redirect::setState(0);
				}
			} else if (isDetectingRing) {
				if (detectedRingColor == "none");
				else if (detectedRingColor == filterOutColor) {
					// Filter out
					if (redirect::getState() == 0) {
						redirect::setState(1);
					}
				} else {
					// Remove filter
					if (redirect::getState() == 1) {
						redirect::setState(0);
					}
				}
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
}
