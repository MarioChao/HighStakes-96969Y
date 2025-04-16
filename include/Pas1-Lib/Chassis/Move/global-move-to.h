#pragma once

#include "Pas1-Lib/Chassis/Base/differential.h"


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


struct turnToFace_params {
	turnToFace_params(
		aespa_lib::units::Length x, aespa_lib::units::Length y,
		bool isReverse = false, double maxTurnVelocity_pct = 100, double centerOffset_tiles = 0
	)
		: x(x), y(y), isReverse(isReverse),
		maxTurnVelocity_pct(maxTurnVelocity_pct), centerOffset_tiles(centerOffset_tiles) {}

	aespa_lib::units::Length x, y;
	bool isReverse = false;
	double maxTurnVelocity_pct = 100;
	double centerOffset_tiles = 0;
	double runTimeout_sec = 3;
};

void turnToFace(base::Differential &chassis, turnToFace_params params, bool async);

extern double _turnToFaceError_degrees;
extern bool _isTurnToFaceSettled;


struct driveToPoint_params {
	driveToPoint_params(
		aespa_lib::units::Length x, aespa_lib::units::Length y,
		bool isReverse = false, double maxVelocity_pct = 100,
		double runTimeout_sec = 3
	)
		: x(x), y(y), isReverse(isReverse), maxVelocity_pct(maxVelocity_pct), runTimeout_sec(runTimeout_sec) {}

	aespa_lib::units::Length x, y;
	bool isReverse = false;
	double maxVelocity_pct = 100, maxTurnVelocity_pct = 100;
	double runTimeout_sec = 3;
};

void driveToPoint(base::Differential &chassis, driveToPoint_params params, bool async);

extern double _linearPathDistanceError;
extern bool _isDriveToPointSettled;


}
}
}
}
