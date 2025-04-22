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

bool antiJamEnabled = true;

bool colorFilterEnabled = true;
const bool disableColorFilter = false;

int resolveState = 0;

std::string filterOutColor = "none";
bool previousIsDetecting = false;
std::string lastDetectedRingColor = "none";

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

			// Check jam
			if (antiJamEnabled) {
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
			} else isStuck = false;

			// Jam control
			if (isStuck && stuckTime.time(sec) > 0.2) {
				// Reverse a little on stuck
				IntakeMotor1.spin(fwd, -12, volt);
				wait(130, msec);
			} else {
				resolveIntake();
			}
		} else {
			/* Store ring */

			resolveState = 1;

			double distanceSensor_detectedDistance = RingDistanceSensor.objectDistance(distanceUnits::mm);
			bool distanceSensor_isDetectingRing = distanceSensor_detectedDistance < 30;

			if (true) {
				if (distanceSensor_isDetectingRing && lastDetectedRingColor != "none" && lastDetectedRingColor != filterOutColor) {
					printf("Distance sensor: store ring\n");
					resolveState = 0;
					isStoringRing = false;
				}
			} else {
				bool isDetectingRing = ringoptical::isDetecting();
				std::string detectedRingColor = ringoptical::getDetectedColor();
				if (isDetectingRing && detectedRingColor != "none") {
					if (detectedRingColor != filterOutColor) {
						resolveState = 0;
						isStoringRing = false;
					}
				}
			}

			resolveIntake();
		}

		wait(5, msec);
	}
}


void preauton() {
	setAntiJam(false);
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

void setAntiJam(bool antiJamState) {
	antiJamEnabled = antiJamState;
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

std::string getFilterOutColor() {
	return filterOutColor;
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
	}
}

bool canControl() {
	return controlState;
}

void setControlState(bool canControl) {
	controlState = canControl;
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

	// Update ring detection
	ringoptical::updateDetection();
	bool isDetectingRing = ringoptical::isDetecting();
	std::string detectedRingColor = ringoptical::getDetectedColor();
	if (detectedRingColor != "none") lastDetectedRingColor = detectedRingColor;

	// Filter out on some detection
	if (colorFilterEnabled && !disableColorFilter) {
		// Sensor data
		double distanceSensor_detectedDistance = RingDistanceSensor.objectDistance(distanceUnits::mm);
		bool distanceSensor_isDetectingRing = distanceSensor_detectedDistance < 30;

		// Newly detected ring
		bool isNewDetected = (!previousIsDetecting && isDetectingRing);
		previousIsDetecting = isDetectingRing;
		// printf("Col: %s distance: %.3f %d\n", lastDetectedRingColor.c_str(), distanceSensor_detectedDistance, distanceSensor_isDetectingRing);
		
		// Filter
		if (true) {
			/* Optical sensor + distance sensor*/
			if (distanceSensor_isDetectingRing) {
				// printf("Col: %s %s\n", lastDetectedRingColor.c_str(), filterOutColor.c_str());
				if (lastDetectedRingColor == "none");
				else if (lastDetectedRingColor == filterOutColor) {
					printf("Distance sensor: filtering out\n");
					wait(175, msec);
					IntakeMotor1.spin(fwd, -12, volt);
					wait(80, msec);
				}
			}
		} else {
			/* Only optical sensor */
			if (isNewDetected) {
				if (detectedRingColor == "none");
				else if (detectedRingColor == filterOutColor) {
					// Filter out
					// printf("Optical sensor: filtering out\n");
					// wait(30, msec);
					// IntakeMotor1.spin(fwd, -5, volt);
					// wait(100, msec);
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
