#include "Mechanics/botIntake.h"

#include "Sensors/ringOptical.h"
#include "Mechanics/redirect.h"
#include "Mechanics/botArm.h"
#include "Utilities/debugFunctions.h"
#include "Aespa-Lib/Winter-Utilities/general.h"
#include "main.h"

// This mechanic is for one-part intake with two motors

namespace {
	void resolveIntake();

	double intakeVelocityPct = 100;

	/* Factors */

	bool colorFilterEnabled = true;
	const bool disableColorFilter = false;

	int resolveState = 0;

	std::string filterOutColor = "none";
	bool previousIsDetecting = false;

	bool isStoringRing = false;

	bool controlState = true;
}


namespace botintake {
	void runThread() {
		timer stuckTime;
		bool isStuck = false;
		while (true) {
			/* Intake loop */

			if (!isStoringRing) {
				/* Normal intake */

				// Detect stuck
				// printf("Int tq: %.3f\n ", IntakeMotor1.torque(Nm));
				if (IntakeMotor1.torque() > 0.35) {
					if (!isStuck) {
						stuckTime.clear();
						isStuck = true;
					}
				} else {
					isStuck = false;
				}

				// Override stuck under certain conditions
				if (botarm::getArmStage() == 1) {
					isStuck = false;
				}

				if (isStuck && stuckTime.time(seconds) > 0.1) {
					// Reverse a little on stuck
					// IntakeMotor1.spin(fwd, -4, volt);
					// wait(100, msec);
					IntakeMotor1.spin(fwd, -4, volt);
					wait(100, msec);
				} else {
					resolveIntake();
				}
			} else {
				/* Store ring */

				resolveState = 1;
				resolveIntake();

				bool isDetectingRing = ringoptical::isDetecting();
				std::string detectedRingColor = ringoptical::getDetectedColor();

				if (isDetectingRing && detectedRingColor != "none") {
					if (detectedRingColor != filterOutColor) {
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
			setState(state);
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
			// Sensor data
			ringoptical::updateDetection();
			bool isDetectingRing = ringoptical::isDetecting();
			std::string detectedRingColor = ringoptical::getDetectedColor();

			// Newly detected ring
			bool isNewDetected = (!previousIsDetecting && isDetectingRing);
			previousIsDetecting = isDetectingRing;
			
			if (disableColorFilter) {
				// if (redirect::getState() == 1) {
					// 	redirect::setState(0);
					// }
			} else if (isNewDetected) {
				if (detectedRingColor == "none");
				else if (detectedRingColor == filterOutColor) {
					// Filter out
					printf("Filtering out\n");
					// wait(50, msec);
					// IntakeMotor1.spin(fwd, 5, volt);
					wait(30, msec);
					IntakeMotor1.spin(fwd, -5, volt);
					wait(200, msec);
					IntakeMotor1.spin(fwd, 11, volt);
					wait(180, msec);
					IntakeMotor1.spin(fwd, -5, volt);
					wait(200, msec);
					return;
					// if (redirect::getState() == 0) {
					// 	redirect::setState(1);
					// }
				} else {
					// Remove filter
					// if (redirect::getState() == 1) {
					// 	redirect::setState(0);
					// }
				}
			}
		}

		// Resolve intake
		switch (resolveState) {
			case 1:
				// Forward
				IntakeMotor1.spin(fwd, aespa_lib::genutil::pctToVolt(intakeVelocityPct), volt);
				IntakeMotor2.spin(fwd, aespa_lib::genutil::pctToVolt(intakeVelocityPct), volt);
				break;
			case -1:
				// Reversed
				IntakeMotor1.spin(fwd, -aespa_lib::genutil::pctToVolt(intakeVelocityPct), volt);
				IntakeMotor2.spin(fwd, -aespa_lib::genutil::pctToVolt(intakeVelocityPct), volt);
				break;
			default:
				IntakeMotor1.stop(brakeType::coast);
				IntakeMotor2.stop(brakeType::coast);
				break;
		}
	}
}
