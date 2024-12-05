#include "AutonUtilities/patienceController.h"

#include <stdio.h>
#include <cmath>

PatienceController::PatienceController(int maxPatience, double minDelta, bool positiveImprovement) {
	maxPatienceLevel = maxPatience;
	this->absoluteMinDelta = fabs(minDelta);
	this->positiveImprovement = positiveImprovement;
	reset();
}

void PatienceController::reset() {
	// Reset patience
	patience = 0;

	// Reset stored value
	if (positiveImprovement) {
		storedValue = -1e9;
	} else {
		storedValue = 1e9;
	}
}

void PatienceController::computePatience(double value) {
	// Calculate improvement
	double delta = value - storedValue;
	bool willResetPatience = positiveImprovement && delta > absoluteMinDelta;
	willResetPatience &= (!positiveImprovement && delta < -absoluteMinDelta);

	// Modify patience
	if (willResetPatience) {
		patience = 0;
	} else {
		patience++;
	}

	// Update value
	if (positiveImprovement && delta > 0) {
		storedValue = value;
	} else if (!positiveImprovement && delta < 0) {
		storedValue = value;
	}
}

void PatienceController::exhaustNow() {
	patience = maxPatienceLevel;
}

bool PatienceController::isExhausted() {
	return patience >= maxPatienceLevel;
}

void PatienceController::printDebug() {
	printf("Pat: %d, val: %.3f\n", patience, storedValue);
}
