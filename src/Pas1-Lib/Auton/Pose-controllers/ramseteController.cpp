#include "Pas1-Lib/Auton/Pose-Controllers/ramseteController.h"

#include <cmath>
#include "AutonUtilities/linegular.h"
#include "Aespa-Lib/Winter-Utilities/angleUtility.h"
#include "Aespa-Lib/Winter-Utilities/generalUtility.h"
#include <stdio.h>

namespace {
	const double defaultB = 0.743; // 2.0 rad^2/m^2 * (1 m / 1.64041995 tiles)^2 = 0.7432
	const double defaultDamp = 0.7;

	double smallScalar = 0.0001;
}

RamseteController::RamseteController()
: RamseteController(defaultB, defaultDamp) {}

RamseteController::RamseteController(double b, double damp) {
	this->b = b;
	this->zeta = damp;
}

void RamseteController::setDirection(bool isReversed) {
	if (isReversed) {
		directionFactor = -1;
	} else {
		directionFactor = 1;
	}
}

std::pair<double, double> RamseteController::getLinegularVelocity(
	Linegular actual, Linegular desired
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(aespa_lib::genutil::toRadians(90 - actual.getThetaPolarAngle_degrees()));

	return getLinegularVelocity(actual, desired, smallScalar * error.getY(), smallScalar * error.getThetaPolarAngle_radians());
}

std::pair<double, double> RamseteController::getLinegularVelocity(
	Linegular actual, Linegular desired,
	double desiredLinearVelocity
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(aespa_lib::genutil::toRadians(90 - actual.getThetaPolarAngle_degrees()));

	return getLinegularVelocity(actual, desired, desiredLinearVelocity, smallScalar * error.getThetaPolarAngle_radians());
}

std::pair<double, double> RamseteController::getLinegularVelocity(
	Linegular actual, Linegular desired,
	double desiredLinearVelocity, double desiredAngularVelocity_radiansPerSecond
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(aespa_lib::genutil::toRadians(90 - actual.getThetaPolarAngle_degrees()));

	// Get value alias
	double v_desired = fabs(desiredLinearVelocity) * directionFactor;
	auto &w_desired = desiredAngularVelocity_radiansPerSecond;
	auto e_right = error.getX();
	auto e_look = error.getY();
	auto e_theta = aespa_lib::genutil::toRadians(aespa_lib::genutil::modRange(error.getThetaPolarAngle_degrees(), 360, -180));
	// printf("ANG ERR: %.f\n", genutil::toDegrees(e_theta));

	// Compute gain value
	// refer to https://wiki.purduesigbots.com/software/control-algorithms/ramsete
	// Note: "ey is equal to -e_right"
	// Note: "the positive θ direction is turning the robot to the right"
	double k = 2 * zeta * sqrt(pow(w_desired, 2) + b * pow(v_desired, 2));

	// Compute output velocities
	double outputLinearVelocity = (v_desired * cos(e_theta)) + (k * e_look);
	double outputAngularVelocity = (w_desired) + (k * e_theta) + (b * v_desired * aespa_lib::angle::sinc(e_theta) * e_right);

	// Return linegular velocities
	std::pair<double, double> result = std::make_pair(outputLinearVelocity, outputAngularVelocity);
	return result;
}
