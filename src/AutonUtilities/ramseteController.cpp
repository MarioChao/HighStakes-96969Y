#include "AutonUtilities/ramseteController.h"

#include <cmath>
#include "AutonUtilities/linegular.h"
#include "Utilities/angleUtility.h"
#include "Utilities/generalUtility.h"
#include <stdio.h>

namespace {
	double smallScalar = 0.1;
}

RamseteController::RamseteController(double b, double damp) {
	this->b = b;
	this->zeta = damp;
}

std::pair<double, double> RamseteController::getLeftRightVelocity_pct(
	Linegular actual, Linegular desired
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(genutil::toRadians(90 - actual.getTheta_degrees()));

	return getLeftRightVelocity_pct(actual, desired, smallScalar * error.getY(), smallScalar * error.getTheta_radians());
}

std::pair<double, double> RamseteController::getLeftRightVelocity_pct(
	Linegular actual, Linegular desired,
	double desiredLinearVelocity
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(genutil::toRadians(90 - actual.getTheta_degrees()));

	return getLeftRightVelocity_pct(actual, desired, desiredLinearVelocity, smallScalar * error.getTheta_radians());
}

std::pair<double, double> RamseteController::getLeftRightVelocity_pct(
	Linegular actual, Linegular desired,
	double desiredLinearVelocity, double desiredAngularVelocity
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(genutil::toRadians(90 - actual.getTheta_degrees()));

	// Get value alias
	auto &v_desired = desiredLinearVelocity;
	auto &w_desired = desiredAngularVelocity;
	auto e_right = error.getX();
	auto e_look = error.getY();
	// auto e_theta = error.getTheta_radians();
	auto e_theta = genutil::toRadians(angle::modRange(error.getTheta_degrees(), 360, -180));
	// printf("ANG ERR: %.f\n", genutil::toDegrees(e_theta));

	// Compute gain value
	// refer to https://wiki.purduesigbots.com/software/control-algorithms/ramsete
	double k = 2 * zeta * sqrt(pow(w_desired, 2) + b * pow(v_desired, 2));

	// Compute output velocities
	double outputLinearVelocity = (v_desired * cos(e_theta)) + (k * e_look);
	double outputAngularVelocity = -(w_desired) + (k * e_theta) - (b * v_desired * angle::sinc(e_theta) * e_right);

	// Compute left and right velocities
	// outputAngularVelocity *= -1; // for polar angles
	double leftVelocity = outputLinearVelocity - outputAngularVelocity;
	double rightVelocity = outputLinearVelocity + outputAngularVelocity;
	// printf("outlin: %.3f, outang: %.3f\n", outputLinearVelocity, outputAngularVelocity);

	// Return velocities
	std::pair<double, double> result = std::make_pair(leftVelocity, rightVelocity);
	return result;
}
