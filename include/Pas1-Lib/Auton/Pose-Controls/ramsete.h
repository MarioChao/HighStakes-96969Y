#pragma once

#include <utility>

class Linegular;


namespace pas1_lib {
namespace auton {
namespace pose_controllers {

class RamseteController {
public:
	RamseteController(double b, double damp);
	RamseteController();

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

}
}
}
