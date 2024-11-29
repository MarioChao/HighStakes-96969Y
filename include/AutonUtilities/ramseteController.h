#pragma once

#include <utility>

class Linegular;

class RamseteController {
public:
	RamseteController(double b = 50.0, double damp = 0.8);

	bool setDirection(bool isReversed);

	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired
	);
	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity
	);
	std::pair<double, double> getLeftRightVelocity_pct(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity, double desiredAngularVelocity_radiansPerSecond
	);

private:
	double b, zeta;
	double directionFactor = 1;
};
