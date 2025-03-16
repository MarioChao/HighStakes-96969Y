#include "Aespa-Lib/Karina-Data-Structures/linegular.h"

#include <cmath>
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

namespace aespa_lib {
namespace datas {


// ---------- Vector2D ----------

Vector2D::Vector2D(double x, double y)
	: x(x), y(y) {}

void Vector2D::rotateBy(double polarRotate_radians) {
	double newX = x * cos(polarRotate_radians) - y * sin(polarRotate_radians);
	double newY = x * sin(polarRotate_radians) + y * cos(polarRotate_radians);
	x = newX;
	y = newY;
}

void Vector2D::rotateExponentialBy(double polarRotate_radians) {
	// Check pose exponential in https://file.tavsys.net/control/controls-engineering-in-frc.pdf
	double newX = x * aespa_lib::angle::sinc(polarRotate_radians) + y * aespa_lib::angle::cosm1_x(polarRotate_radians);
	double newY = x * -aespa_lib::angle::cosm1_x(polarRotate_radians) + y * aespa_lib::angle::sinc(polarRotate_radians);
	x = newX;
	y = newY;
}

double Vector2D::getMagnitude() {
	return aespa_lib::genutil::l2Norm({ x, y });
}


// ---------- Linegular ----------

Linegular::Linegular(Vector2D position, double polarTheta_degrees)
	: position(position),
	theta_degrees(polarTheta_degrees) {}

Linegular::Linegular(double x, double y, double polarTheta_degrees)
	: Linegular(Vector2D(x, y), polarTheta_degrees) {}

Vector2D Linegular::getPosition() {
	return position;
}

double Linegular::getX() {
	return position.x;
}

double Linegular::getY() {
	return position.y;
}

double Linegular::getThetaPolarAngle_degrees() {
	return theta_degrees;
}

double Linegular::getThetaPolarAngle_radians() {
	return aespa_lib::genutil::toRadians(theta_degrees);
}

double Linegular::getXYMagnitude() {
	return position.getMagnitude();
}

void Linegular::setPosition(Vector2D newPosition) {
	position = newPosition;
}

void Linegular::setPosition(double x, double y) {
	setPosition(Vector2D(x, y));
}

void Linegular::rotateXYBy(double polarRotate_radians) {
	position.rotateBy(polarRotate_radians);
}

void Linegular::rotateExponentialBy(double polarRotate_radians) {
	position.rotateExponentialBy(polarRotate_radians);
}

Linegular Linegular::operator+(Linegular &other) {
	return Linegular(getX() + other.getX(), getY() + other.getY(), theta_degrees + other.theta_degrees);
}

Linegular Linegular::operator-(Linegular &other) {
	return Linegular(getX() - other.getX(), getY() - other.getY(), theta_degrees - other.theta_degrees);
}

Linegular Linegular::operator*(double value) {
	return Linegular(getX() * value, getY() * value, theta_degrees);
}

Linegular Linegular::operator/(double value) {
	return operator*(1 / value);
}

}
}
