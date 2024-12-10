#include "Utilities/generalUtility.h"

#include <cmath>
#include <algorithm>

namespace genutil {
	double modRange(double num, double mod, double min) {
		// Offset from minimum
		double ret = fmod(num - min, mod);
		// Get positive
		if (ret < 0) ret += fabs(mod);
		// Offset to minimum
		ret += min;
		return ret;
	}

	double clamp(double value, double min, double max) {
		return fmin(max, fmax(min, value));
	}

	double pctToVolt(double pct) {
		return pct * 12.0 / 100.0;
	}

	int signum(double value) {
		if (value > 0) return 1;
		if (value == 0) return 0;
		return -1;
	}

	bool isWithin(double value, double target, double withinRange) {
		return fabs(value - target) <= withinRange;
	}

	double euclideanDistance(std::vector<double> point1, std::vector<double> point2) {
		int dimCount = std::min((int) point1.size(), (int) point2.size());
		double squaredSum = 0;
		for (int i = 0; i < dimCount; i++) {
			double delta = point1[i] - point2[i];
			squaredSum += delta * delta;
		}
		return sqrt(squaredSum);
	}

	double toRadians(double degrees) {
		return degrees * M_PI / 180.0;
	}

	double toDegrees(double radians) {
		return radians * 180.0 / M_PI;
	}

	double rangeMap(double x, double inMin, double inMax, double outMin, double outMax) {
		return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
	}

	double getScaleFactor(double scaleToMax, std::initializer_list<double> list) {
		scaleToMax = fabs(scaleToMax);
		return scaleToMax / fmax(scaleToMax, maxAbsolute(list));
	}

	double maxAbsolute(std::initializer_list<double> list) {
		// Get maximum absolute
		double result = 0;
		for (double element : list) {
			double value = fabs(element);
			if (value > result) {
				result = value;
			}
		}
		return result;
	}

	double getAverage(std::vector<double> list) {
		double sum = 0;
		int count = 0;
		for (double element : list) {
			sum += element;
			count++;
		}

		double result = sum / count;
		return result;
	}
}
