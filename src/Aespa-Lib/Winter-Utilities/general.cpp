#include "Aespa-Lib/Winter-Utilities/general.h"

#include <cmath>
#include <algorithm>

namespace aespa_lib {
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

double voltToPct(double volt) {
	return volt * 100.0 / 12.0;
}

int signum(double value) {
	if (value > 0) return 1;
	if (value == 0) return 0;
	return -1;
}

bool isWithin(double value, double target, double withinRange) {
	return fabs(value - target) <= withinRange;
}

double toRadians(double degrees) {
	return degrees * M_PI / 180.0;
}

double toDegrees(double radians) {
	return radians * 180.0 / M_PI;
}

double lerp(double value1, double value2, double t) {
	return value1 + (value2 - value1) * t;
}
double rangeMap(double x, double inMin, double inMax, double outMin, double outMax) {
	return lerp(outMin, outMax, (x - inMin) / (inMax - inMin));
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

std::vector<double> getAbsolute(std::vector<double> list) {
	std::vector<double> result;
	for (double element : list) {
		result.push_back(fabs(element));
	}
	return result;
}

std::vector<double> multiplyVector(std::vector<double> list, double scale) {
	std::vector<double> result;
	for (double element : list) {
		result.push_back(element * scale);
	}
	return result;
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

double l2Norm(std::vector<double> point) {
	return euclideanDistance(point, std::vector<double>((int) point.size(), 0));
}

std::pair<bool, double> getArcRadius_inches(double arcLength_inches, double rotatedAngle_polarDegrees) {
	if (isWithin(rotatedAngle_polarDegrees, 0, 1e-7)) {
		return { false, 1e9 };
	}
	double arcRadius_inches = (arcLength_inches / rotatedAngle_polarDegrees);
	return { true, arcRadius_inches };
}

double getChordLength_inches(double arcRadius_inches, double rotatedAngle_polarDegrees) {
	double chordLength_inches = 2 * std::sin(rotatedAngle_polarDegrees / 2) * arcRadius_inches;
	return chordLength_inches;
}

std::pair<double, std::vector<double>> integratePolynomial(std::vector<double> value_dY_dX, double dX) {
	// Integrate
	const int polyDegree = (int) value_dY_dX.size();
	std::vector<double> integrated_dY_dX = value_dY_dX;
	std::vector<double> result_dY_dX = value_dY_dX, dXSign;
	double deltaValue = 0;
	for (int degree = 0; degree <= polyDegree; degree++) {
		for (int term = 0; term < degree; term++) {
			int id = polyDegree - 1 - term;

			// Integrate term
			integrated_dY_dX[id] = integrated_dY_dX[id] * dX / (degree - term);

			// Add to result
			if (degree == polyDegree) deltaValue += integrated_dY_dX[id];
			else result_dY_dX[polyDegree - 1 - degree] += integrated_dY_dX[id];
		}
	}
	return {deltaValue, result_dY_dX};
}

}
}
