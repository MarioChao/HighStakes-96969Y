#include "Aespa-Lib/Giselle-Geometry/vector-2d.hpp"

#include <cmath>
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace aespa_lib {
namespace geometry {


Vector2D::Vector2D(double x, double y)
	: x(x), y(y) {}

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


}
}
