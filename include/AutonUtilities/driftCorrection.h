#pragma once

#include "main.h"

class DriftCorrection {
public:
	DriftCorrection(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift);
	void setInitial();
	void correct();
private:
	inertial *sensor;
	double perClockwiseRevolutionDrift, perCCWRevolutionDrift;
	double storedInitialRotation;
};
