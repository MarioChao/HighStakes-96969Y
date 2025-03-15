#include "Pas1-Lib/Auton/Control-Loops/slew.h"

#include "Aespa-Lib/Winter-Utilities/general.h"


namespace pas1_lib {
namespace auton {
namespace control_loops {

SlewController::SlewController(double maxChangeRate_valPerSec)
	: maxChangeRate_valPerSec(maxChangeRate_valPerSec) {
	reset();
}

void SlewController::reset(bool ignoreFirst) {
	if (ignoreFirst) this->ignoreFirst = ignoreFirst;
	slewTimer.reset();
}

void SlewController::computeFromTarget(double targetValue) {
	if (ignoreFirst) {
		storedValue = targetValue;
		ignoreFirst = false;
		return;
	}

	if (maxChangeRate_valPerSec < 0) {
		storedValue = targetValue;
		return;
	}

	// Get max change
	double deltaTime_sec = slewTimer.time(seconds);
	double maxChange = fabs(maxChangeRate_valPerSec * deltaTime_sec);

	// Constrain change
	double oldChange = targetValue - storedValue;
	double realChange = aespa_lib::genutil::clamp(oldChange, -maxChange, maxChange);

	// Store new value
	storedValue += realChange;
}

double SlewController::getValue() {
	return storedValue;
}

}
}
}
