#include "Aespa-Lib/Giselle-Geometry/vector-2d.hpp"

#include <cmath>
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace aespa_lib {
namespace geometry {


Vector2D::Vector2D(double x, double y)
	: x(x), y(y) {}

Vector2D Vector2D::fromPolar(units::PolarAngle angle, double magnitude) {
	double radians = angle.polarRad();
	return Vector2D(magnitude * std::cos(radians), magnitude * std::sin(radians));
}

void Vector2D::rotateBy(units::PolarAngle rotation) {
	double radians = rotation.polarRad();
	double newX = x * cos(radians) - y * sin(radians);
	double newY = x * sin(radians) + y * cos(radians);
	x = newX;
	y = newY;
}

void Vector2D::rotateExponentialBy(units::PolarAngle rotation) {
	// Check pose exponential in https://file.tavsys.net/control/controls-engineering-in-frc.pdf
	double radians = rotation.polarRad();
	double newX = x * aespa_lib::angle::sinc(radians) + y * aespa_lib::angle::cosm1_x(radians);
	double newY = x * -aespa_lib::angle::cosm1_x(radians) + y * aespa_lib::angle::sinc(radians);
	x = newX;
	y = newY;
}

double Vector2D::getMagnitude() {
	return aespa_lib::genutil::l2Norm({ x, y });
}

Vector2D Vector2D::getNormalized() {
	double magnitude = getMagnitude();
	return Vector2D(x / magnitude, y / magnitude);
}

Vector2D Vector2D::operator-() {
	return Vector2D(-x, -y);
}

Vector2D Vector2D::operator+(Vector2D other) {
	return Vector2D(x + other.x, y + other.y);
}

Vector2D Vector2D::operator-(Vector2D other) {
	return *this + -other;
}

double Vector2D::cross_scalar(Vector2D other) {
	return x * other.y - y * other.x;
}

double Vector2D::dot(Vector2D other) {
	return x * other.x + y * other.y;
}

units::PolarAngle Vector2D::angleFrom(Vector2D other) {
	double this_radians = std::atan2(y, x);
	double other_radians = std::atan2(other.y, other.x);
	return units::operator ""_polarRad((long double) this_radians - other_radians);
}


}
}
