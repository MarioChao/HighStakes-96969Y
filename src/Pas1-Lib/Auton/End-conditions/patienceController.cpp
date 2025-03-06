#include "AutonUtilities/patienceController.h"

#include <stdio.h>
#include <cmath>

PatienceController::PatienceController(int maxPatience, double minDelta, bool positiveImprovement, int delayComputePatience) {
	maxPatienceLevel = maxPatience;
	this->absoluteMinDelta = std::abs(minDelta);
	this->positiveImprovement = positiveImprovement;
	delayComputePatienceLevel = delayComputePatience;
	reset();
}

void PatienceController::reset() {
	// Reset patience
	patience = 0;
	delayedCalls = 0;

	// Reset stored value
	if (positiveImprovement) {
		storedValue = -1e9;
	} else {
		storedValue = 1e9;
	}
}

void PatienceController::computePatience(double value) {
	// Resolve delayed calls
	if (delayedCalls < delayComputePatienceLevel) {
		delayedCalls++;
		return;
	}

	// Calculate improvement
	double delta = value - storedValue;
	bool willResetPatience = positiveImprovement && delta > absoluteMinDelta;
	willResetPatience |= (!positiveImprovement && delta < -absoluteMinDelta);

	// Modify patience
	if (willResetPatience) {
		patience = 0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                
		// Update value
		storedValue = value;
	} else {
		patience++;
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
