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
			} else if (190 <= RingOpticalSensor.hue() && RingOpticalSensor.hue() <= 230) {
				detectedRingColor = "blue";
				// debug::printOnController("Blue ring");
			}

			// Update detecting ring
			isDetectingRing = RingOpticalSensor.isNearObject();

			/* Intake loop */

			// Normal intake
			if (false) {
				// Detect stuck
				if (IntakeMotor2.torque() > 0.41) {
					if (!isStuck) {
						stuckTime.clear();
						isStuck = true;
					}
				} else {
					isStuck = false;
				}
				isStuck = false;

				// Reverse on stuck
				if (isStuck && stuckTime.value() > 0.08) {
					resolveState = -1;
					resolveIntake();
					task::sleep(300);
				} else {
					resolveIntake();
				}
			} else {
				resolveIntake();
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

	void setColorFiltering(bool isEnabled) {
		colorFilterEnabled = isEnabled;
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

	void setFilterColor(std::string colorText) {
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
		if (colorFilterEnabled) {
			if (disableColorFilter) {
				if (redirect::getState() == 1) {
					redirect::setState(0);
				}
			} else if (isDetectingRing) {
				if (detectedRingColor == filterOutColor) {
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
