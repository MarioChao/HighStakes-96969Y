#pragma once

#include <utility>

class Linegular;

class RamseteController {
public:
	RamseteController();
	RamseteController(double b, double damp);

	void setDirection(bool isReversed);

	std::pair<double, double> getLinegularVelocity(
		Linegular actual, Linegular desired
	);
	std::pair<double, double> getLinegularVelocity(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity
	);
	std::pair<double, double> getLinegularVelocity(
		Linegular actual, Linegular desired,
		double desiredLinearVelocity, double desiredAngularVelocity_radiansPerSecond
	);

private:
	double b, zeta;
	double directionFactor = 1;
};
