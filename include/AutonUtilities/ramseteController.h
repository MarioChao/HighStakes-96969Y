#pragma once

#include <utility>

class Linegular;

class RamseteController {
public:
	RamseteController(double b = 2.0, double damp = 0.7);

	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired
	);
	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity
	);
	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity, double desiredAngularVelocity
	);

private:
	double b, zeta;
};
