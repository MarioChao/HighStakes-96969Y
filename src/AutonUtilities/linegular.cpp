#include "AutonUtilities/linegular.h"

#include <cmath>
#include "Utilities/angleUtility.h"
#include "Utilities/generalUtility.h"

Linegular::Linegular(double x, double y, double polarTheta_degrees) {
	this->x = x;
	this->y = y;
	this->theta = polarTheta_degrees;
}

double Linegular::getX(){
	return x;
}

double Linegular::getY(){
	return y;
}

double Linegular::getTheta_degrees(){
	return theta;
}

double Linegular::getTheta_radians() {
	return genutil::toRadians(theta);
}

void Linegular::rotateXYBy(double polarRotate_radians) {
	double newX = x * cos(polarRotate_radians) - y * sin(polarRotate_radians);
	double newY = x * sin(polarRotate_radians) + y * cos(polarRotate_radians);
	x = newX;
	y = newY;
}

void Linegular::rotateExponentialBy(double polarRotate_radians) {
	// Check pose exponential in https://file.tavsys.net/control/controls-engineering-in-frc.pdf
	double newX = x * angle::sinc(polarRotate_radians) + y * angle::cosm1_x(polarRotate_radians);
	double newY = x * -angle::cosm1_x(polarRotate_radians) + y * angle::sinc(polarRotate_radians);
	x = newX;
	y = newY;
}

Linegular Linegular::operator+(Linegular &other) {
	return Linegular(x + other.x, y + other.y, theta + other.theta);
}

Linegular Linegular::operator-(Linegular &other) {
	return Linegular(x - other.x, y - other.y, theta - other.theta);
}
