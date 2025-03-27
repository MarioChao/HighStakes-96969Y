#pragma once

#include "Pas1-Lib/Basic-Control/Chassis/differential.h"
#include <vector>


namespace pas1_lib {
namespace basic_control {
namespace move_chassis {
namespace local {


struct driveAndTurn_params {
	driveAndTurn_params(
		double distance_tiles, double targetAngle_polarDegrees,
		std::vector<std::pair<double, double>> velocityConstraint_tiles_pct,
		double maxTurnVelocity_pct = 100
	)
		: distance_tiles(distance_tiles), targetAngle_polarDegrees(targetAngle_polarDegrees),
		velocityConstraint_tiles_pct(velocityConstraint_tiles_pct),
		maxTurnVelocity_pct(maxTurnVelocity_pct) {}

	driveAndTurn_params(
		double distance_tiles, double targetAngle_polarDegrees,
		double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100
	)
		: driveAndTurn_params(distance_tiles, targetAngle_polarDegrees, {{0, maxVelocity_pct}}, maxTurnVelocity_pct) {}

	double distance_tiles;
	double targetAngle_polarDegrees;
	std::vector<std::pair<double, double>> velocityConstraint_tiles_pct;
	double maxTurnVelocity_pct = 100;
	double runTimeout_sec = 3;
};

void driveAndTurn(chassis::Differential &chassis, driveAndTurn_params params, bool async);

extern double _driveDistanceError_tiles;
extern bool _isDriveAndTurnSettled;


}
}
}
}
