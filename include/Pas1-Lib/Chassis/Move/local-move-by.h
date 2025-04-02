#pragma once

#include "Aespa-Lib/Winter-Utilities/units.h"
#include "Pas1-Lib/Chassis/Base/differential.h"
#include <vector>


namespace pas1_lib {
namespace chassis {
namespace move {
namespace local {


struct turnToAngle_params {
	turnToAngle_params(
		aespa_lib::units::PolarAngle targetAngle,
		double maxTurnVelocity_pct = 100,
		double centerOffset_tiles = 0,
		double runTimeout_sec = 3
	)
		: targetAngle(targetAngle),
		maxTurnVelocity_pct(maxTurnVelocity_pct),
		centerOffset_tiles(centerOffset_tiles),
		runTimeout_sec(runTimeout_sec) {}

	aespa_lib::units::PolarAngle targetAngle;
	double maxTurnVelocity_pct = 100;
	double centerOffset_tiles = 0;
	double runTimeout_sec = 3;
};

void turnToAngle(base::Differential &chassis, turnToAngle_params params, bool async);

extern double _turnAngleError_degrees;
extern bool _isTurnToAngleSettled;


struct driveAndTurn_params {
	driveAndTurn_params(
		aespa_lib::units::Length distance,
		aespa_lib::units::PolarAngle targetAngle,
		std::vector<std::pair<double, double>> velocityConstraint_tiles_pct,
		double maxTurnVelocity_pct = 100,
		double runTimeout_sec = 3
	)
		: distance(distance),
		targetAngle(targetAngle),
		velocityConstraint_tiles_pct(velocityConstraint_tiles_pct),
		maxTurnVelocity_pct(maxTurnVelocity_pct),
		runTimeout_sec(runTimeout_sec) {}

	driveAndTurn_params(
		aespa_lib::units::Length distance,
		aespa_lib::units::PolarAngle targetAngle,
		double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100
	)
		: driveAndTurn_params(distance, targetAngle, { {0, maxVelocity_pct} }, maxTurnVelocity_pct) {}

	aespa_lib::units::Length distance;
	aespa_lib::units::PolarAngle targetAngle;
	std::vector<std::pair<double, double>> velocityConstraint_tiles_pct;
	double maxTurnVelocity_pct = 100;
	double runTimeout_sec = 3;
};

void driveAndTurn(base::Differential &chassis, driveAndTurn_params params, bool async);

extern double _driveDistanceError_tiles;
extern bool _isDriveAndTurnSettled;


}
}
}
}
