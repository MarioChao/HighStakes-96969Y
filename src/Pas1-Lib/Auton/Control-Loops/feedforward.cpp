#include "Pas1-Lib/Auton/Control-loops/feedforward.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include <cmath>

namespace pas1_lib {
namespace auton {
namespace control_loops {


/* ---------- Simple Feedforward ----- */

SimpleFeedforward::SimpleFeedforward(double kS, double kV, double kA, double period_sec)
	: kS(kS), kV(kV), kA(kA), period_sec(period_sec) {}

double SimpleFeedforward::calculateDiscrete(double currentVelocity, double nextVelocity) {
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

void SimpleFeedforward::computeFromMotion(double velocity, double acceleration) {
	this->velocity = velocity;
	this->acceleration = acceleration;
}

double SimpleFeedforward::getValue(bool useS, bool useV, bool useA) {
	// Deadband kS
	double valS = 0;
	if (useS) {
		bool isSpeedingUp = aespa_lib::genutil::signum(velocity) == aespa_lib::genutil::signum(acceleration);
		if (1e-5 < std::fabs(velocity) && std::fabs(velocity) < 0.05 && isSpeedingUp) {
			valS = kS * aespa_lib::genutil::signum(velocity);
		}
	}

	// Others
	double valV = useV ? (velocity * kV) : 0;
	double valA = useA ? (acceleration * kA) : 0;

	return valS + valV + valA;
}


/* ---------- Arm Feedforward ----- */

ArmFeedforward::ArmFeedforward(double kS, double kG, double kV, double kA, double period_sec)
	: kS(kS), kG(kG), kV(kV), kA(kA), period_sec(period_sec) {}

double ArmFeedforward::calculateFromMotion(double elevationAngle_rad, double angularVelocity, double angularAcceleration) {
	double valS = kS * aespa_lib::genutil::signum(angularVelocity);
	double valG = kG * std::cos(elevationAngle_rad);
	double valV = kV * angularVelocity;
	double valA = kA * angularAcceleration;
	return valS + valG + valV + valA;
}


}
}
}
