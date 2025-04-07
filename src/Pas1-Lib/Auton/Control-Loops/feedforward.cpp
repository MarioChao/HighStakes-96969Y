#include "Pas1-Lib/Auton/Control-loops/feedforward.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>

namespace pas1_lib {
namespace auton {
namespace control_loops {

ForwardController::ForwardController(double kS, double kV, double kA, double period_sec)
: kS(kS), kV(kV), kA(kA), period_sec(period_sec) {}

double ForwardController::calculateDiscrete(double currentVelocity, double nextVelocity) {
	// Refer to https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/algorithms.md
	if (kA < 1e-9) {
		return kS * aespa_lib::genutil::signum(nextVelocity) + kV * nextVelocity;
	} else {
		double A = -kV / kA;
		double B = 1.0 / kA;
		double A_d = std::exp(A * period_sec);
		double B_d = (
			std::fabs(A) < 1e-9
			? B * period_sec
			: 1.0 / A * (A_d - 1.0) * B
		);
		return (
			kS * aespa_lib::genutil::signum(currentVelocity)
			+ 1.0 / B_d * (nextVelocity - A_d * currentVelocity)
		);
	}
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
			valS = kS * aespa_lib::genutil::signum(velocity);
		}
	}

	// Others
	double valV = useV ? (velocity * kV) : 0;
	double valA = useA ? (acceleration * kA) : 0;

	return valV + valA + valS;
}

}
}
}
