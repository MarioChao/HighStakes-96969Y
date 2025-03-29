#pragma once

#include "Pas1-Lib/Chassis/Base/differential.h"


namespace pas1_lib {
namespace chassis {
namespace move {
namespace global {


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
