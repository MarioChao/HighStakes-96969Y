#pragma once

namespace pas1_lib {
namespace basic_control {
namespace global {

struct driveToPoint_params {
	double x_tiles;
	double y_tiles;
	bool isReverse = false;
	double maxVelocity_pct = 100, maxTurnVelocity_pct = 100;
	double runTimeout_sec = 3;
};

void driveToPoint(driveToPoint_params params);

extern double _linearPathDistanceError;
extern double _targetX, _targetY;
extern bool _isReverseHeading;
extern double _maxVelocity_pct, _maxTurnVelocity_pct;
extern double _runTimeout;
extern bool _isDriveTurnSettled;

}
}
}
