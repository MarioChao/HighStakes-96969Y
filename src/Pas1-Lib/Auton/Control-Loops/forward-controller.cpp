#include "Pas1-Lib/Auton/Control-loops/forward-controller.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>

namespace pas1_lib {
namespace auton {
namespace control_loops {

ForwardController::ForwardController(double kS, double kV, double kA) {
	kStat = kS;
	kVelo = kV, kAcel = kA;
}

void ForwardController::computeFromMotion(double velocity, double acceleration) {
	this->velocity = velocity;
	this->acceleration = acceleration;
}

double ForwardController::getValue(bool useS, bool useV, bool useA) {
	// Deadband kS
	double valS = 0;
	if (useS) {
		bool isSpeedingUp = aespa_lib::genutil::signum(velocity) == aespa_lib::genutil::signum(acceleration);
		if (1e-5 < fabs(velocity) && fabs(velocity) < 0.05 && isSpeedingUp) {
			valS = kStat;
		}
	}

	// Others
	double valV = useV ? (velocity * kVelo) : 0;
	double valA = useA ? (acceleration * kAcel) : 0;

	return valV + valA + valS;
}

}
}
}
