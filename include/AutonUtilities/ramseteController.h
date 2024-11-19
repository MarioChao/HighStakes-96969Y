#pragma once

#include <utility>

class Linegular;

class RamseteController {
public:
	RamseteController(double b = 0.1, double damp = 0.2);

	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity, double desiredAngularVelocity,
		bool isAnglesPolar = true
	);

private:
	double b, zeta;
};
