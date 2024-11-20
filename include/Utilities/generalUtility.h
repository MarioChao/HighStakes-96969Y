#pragma once

#include <initializer_list>

namespace genutil {
	double clamp(double value, double min, double max);
	double pctToVolt(double pct);
	int signum(double value);
	bool isWithin(double value, double target, double withinRange);

	double toRadians(double degrees);
	double toDegrees(double radians);

	double rangeMap(double x, double inMin, double inMax, double outMin, double outMax);
	double getScaleFactor(double scaleToMax, std::initializer_list<double> list);

	double maxAbsolute(std::initializer_list<double> list);
}
