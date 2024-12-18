#include "AutonUtilities/driftCorrection.h"

DriftCorrection::DriftCorrection(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift) {
	this->sensor = &sensor;
	this->perClockwiseRevolutionDrift = perClockwiseRevolutionDrift;
	this->perCCWRevolutionDrift = perCCWRevolutionDrift;
	_onInit();
}

void DriftCorrection::_onInit() {
	storedInitialRotation = sensor->rotation(deg);
	correctedRotation = storedInitialRotation;
}

void DriftCorrection::setInitial() {
	storedInitialRotation = sensor->rotation();
}

void DriftCorrection::correct() {
	// Calculate change in rotation
	double nowInitialRotation = sensor->rotation();
	double deltaRotation = nowInitialRotation - storedInitialRotation;

	// Calculate drifted rotation
	double addRotation = deltaRotation / 360.0 * ((deltaRotation > 0) ? (-perClockwiseRevolutionDrift) : (perCCWRevolutionDrift));
	double newRotation = nowInitialRotation + addRotation;
	correctedRotation += addRotation;

	// Update 
	sensor->setRotation(newRotation, degrees);
	storedInitialRotation = nowInitialRotation;
}

double DriftCorrection::getRotation() {
	// return correctedRotation;
	return sensor->rotation(deg);
}
