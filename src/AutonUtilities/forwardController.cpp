#include "AutonUtilities/forwardController.h"
#include <cmath>

ForwardController::ForwardController(double kV, double kA, double kS) {
	kVelo = kV, kAcel = kA, kStat = kS;
}

void ForwardController::computeFromMotion(double velocity, double acceleration) {
	this->velocity = velocity;
	this->acceleration = acceleration;
}

double ForwardController::getValue(bool useV, bool useA, bool useS) {
	double valV = useV ? (velocity * kVelo) : 0;
	double valA = useA ? (acceleration * kAcel) : 0;
	double valS = useS ? (kStat) : 0;

	// Deadband kS
	if (fabs(velocity) > 0.05) valS = 0;

	return valV + valA + valS;
}
