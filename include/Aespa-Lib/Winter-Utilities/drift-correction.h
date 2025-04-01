#pragma once

#include "vex.h"


namespace aespa_lib {
namespace util {


class DriftCorrection {
public:
	// Revolution drift in degrees
	DriftCorrection(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift);

	void setInitial();
	void correct();
	double getRotation();

private:
	inertial *sensor;
	double perClockwiseRevolutionDrift, perCCWRevolutionDrift;
	double storedInitialRotation;
};


}
}
