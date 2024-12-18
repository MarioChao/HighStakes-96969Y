#pragma once

#include "main.h"

class DriftCorrection {
public:
	// Reolution drift in degrees
	DriftCorrection(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift);

	void _onInit();

	void setInitial();
	void correct();
	double getRotation();

private:
	inertial *sensor;
	double perClockwiseRevolutionDrift, perCCWRevolutionDrift;
	double storedInitialRotation;
	double correctedRotation;
};
