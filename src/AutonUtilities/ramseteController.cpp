#include "AutonUtilities/ramseteController.h"

#include <cmath>
#include "AutonUtilities/linegular.h"
#include "Utilities/angleFunctions.h"
#include "Utilities/generalUtility.h"
#include <stdio.h>

RamseteController::RamseteController(double b, double damp) {
	this->b = b;
	this->zeta = damp;
}

std::pair<double, double> RamseteController::getLeftRightVelocity_pct(
	Linegular actual, Linegular desired,
	double desiredLinearVelocity, double desiredAngularVelocity,
	bool isAnglesPolar
) {
	// Get local error
	Linegular error = desired - actual;
	error.rotateXYBy(genutil::toRadians(90 - actual.getTheta_degrees()));

	// Compute gain value
	// refer to https://wiki.purduesigbots.com/software/control-algorithms/ramsete
	double k = 2 * zeta * sqrt(
		pow(desiredAngularVelocity, 2) +
		b * pow(desiredLinearVelocity, 2)
	);

	// Compute output velocities
	double outputLinearVelocity = desiredLinearVelocity * cos(error.getTheta_radians()) + k * error.getY();
	double outputAngularVelocity = desiredAngularVelocity + k * error.getTheta_radians() + (
		b * desiredLinearVelocity * angle::sinc(error.getTheta_radians()) * error.getX()
	);

	// Compute left and right velocities
	if (isAnglesPolar) {
		outputAngularVelocity *= -1;
	}
	double leftVelocity = outputLinearVelocity - outputAngularVelocity;
	double rightVelocity = outputLinearVelocity + outputAngularVelocity;

	// Return velocities
	std::pair<double, double> result = std::make_pair(leftVelocity, rightVelocity);
	return result;
}
