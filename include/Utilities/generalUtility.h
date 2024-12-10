#pragma once

#include <initializer_list>
#include <vector>

namespace genutil {
	/**
	 * @brief Return the modulo of a number within a range.
	 * 
	 * @param num The dividend of the modulo.
	 * @param mod The divisor of the modulo.
	 * @param min The smallest possible number of the modulo range.
	 * @return double num % mod in the range [min, min + mod).
	 */
	double modRange(double num, double mod, double min);
	double clamp(double value, double min, double max);

	double pctToVolt(double pct);

	int signum(double value);
	bool isWithin(double value, double target, double withinRange);

	double euclideanDistance(std::vector<double> point1, std::vector<double> point2);

	double toRadians(double degrees);
	double toDegrees(double radians);

	double rangeMap(double x, double inMin, double inMax, double outMin, double outMax);

	double getScaleFactor(double scaleToMax, std::initializer_list<double> list);
	double maxAbsolute(std::initializer_list<double> list);

	double getAverage(std::vector<double> list);
}
