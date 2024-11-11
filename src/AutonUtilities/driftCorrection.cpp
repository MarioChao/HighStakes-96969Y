#include "AutonUtilities/driftCorrection.h"

DriftCorrection::DriftCorrection(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift) {
	this -> sensor = &sensor;
	this -> perClockwiseRevolutionDrift = perClockwiseRevolutionDrift;
	this -> perCCWRevolutionDrift = perCCWRevolutionDrift;
}

void DriftCorrection::setInitial() {
	storedInitialRotation = sensor -> rotation();
}

void DriftCorrection::correct() {
	// Calculate change in rotation
	double nowInitialRotation = sensor -> rotation();
	double deltaRotation = nowInitialRotation - storedInitialRotation;

	// Calculate drifted rotation
	double addRotation = deltaRotation / 360.0 * ((deltaRotation > 0) ? (-perClockwiseRevolutionDrift) : (perCCWRevolutionDrift));
	double newRotation = nowInitialRotation + addRotation;
	sensor -> setRotation(newRotation, degrees);
}
