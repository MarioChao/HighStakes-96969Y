#include "Aespa-Lib/Winter-Utilities/drift-correction.h"


namespace aespa_lib {
namespace util {


DriftCorrection::DriftCorrection(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift)
	: sensor(&sensor),
	perClockwiseRevolutionDrift(perClockwiseRevolutionDrift),
	perCCWRevolutionDrift(perCCWRevolutionDrift) {
	storedInitialRotation = 0;
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

	// Update 
	sensor->setRotation(newRotation, degrees);
	storedInitialRotation = nowInitialRotation;
}

double DriftCorrection::getRotation() {
	return sensor->rotation(deg);
}


}
}
