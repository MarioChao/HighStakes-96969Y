#pragma once

#include <initializer_list>
#include <vector>
#include <functional>

namespace aespa_lib {
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
double voltToPct(double volt);

int signum(double value);
bool isWithin(double value, double target, double withinRange);

double toRadians(double degrees);
double toDegrees(double radians);

double lerp(double value1, double value2, double t);
double rangeMap(double x, double inMin, double inMax, double outMin, double outMax);

double getScaleFactor(double scaleToMax, std::initializer_list<double> list);
double maxAbsolute(std::initializer_list<double> list);

double getAverage(std::vector<double> list);

std::vector<double> getAbsolute(std::vector<double> list);
std::vector<double> multiplyVector(std::vector<double> list, double scale);

double euclideanDistance(std::vector<double> point1, std::vector<double> point2);
double l2Norm(std::vector<double> point);

std::pair<bool, double> getArcRadius_inches(double arcLength_inches, double rotatedAngle_polarDegrees);
double getChordLength_inches(double arcRadius_inches, double rotatedAngle_polarDegrees);

std::pair<double, std::vector<double>> integratePolynomial(
	std::vector<double> value_dY_dX, double dX,
	bool isSimple = false
);

std::pair<bool, double> newtonsMethod(
	std::function<std::pair<double, double>(double)> f_fprime_function,
	double startX = 0,
	double xSkip_step = 0.1, int skipStep_sign = 1
);

}
}
