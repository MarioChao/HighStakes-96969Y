#pragma once

#include "Pas1-Lib/Chassis/Base/differential.h"


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


struct turnToFace_params {
	turnToFace_params(double x_tiles, double y_tiles, bool isReverse = false, double maxTurnVelocity_pct = 100, double centerOffset_tiles = 0)
		: x_tiles(x_tiles), y_tiles(y_tiles), isReverse(isReverse),
		maxTurnVelocity_pct(maxTurnVelocity_pct), centerOffset_tiles(centerOffset_tiles) {}

	double x_tiles;
	double y_tiles;
	bool isReverse = false;
	double maxTurnVelocity_pct = 100;
	double centerOffset_tiles = 0;
	double runTimeout_sec = 3;
};

void turnToFace(base::Differential &chassis, turnToFace_params params, bool async);

extern double _turnToFaceError_degrees;
extern bool _isTurnToFaceSettled;


struct driveToPoint_params {
	driveToPoint_params(double x_tiles, double y_tiles, bool isReverse = false, double maxVelocity_pct = 100)
		: x_tiles(x_tiles), y_tiles(y_tiles), isReverse(isReverse), maxVelocity_pct(maxVelocity_pct) {}

	double x_tiles;
	double y_tiles;
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
