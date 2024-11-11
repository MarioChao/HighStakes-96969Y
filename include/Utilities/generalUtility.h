#pragma once

namespace genutil {
	double clamp(double value, double min, double max);
	double pctToVolt(double pct);
	int signum(double value);
	bool isWithin(double value, double target, double withinRange);
	double toRadians(double degrees);
	double toDegrees(double radians);
}
