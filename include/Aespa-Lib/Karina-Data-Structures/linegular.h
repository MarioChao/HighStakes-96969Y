#pragma once

#include "Aespa-Lib/Winter-Utilities/units.h"
#include "Aespa-Lib/Giselle-Geometry/vector-2d.hpp"


namespace aespa_lib {
namespace datas {

// Class containing 2D position & rotation data
struct Linegular {
	Linegular(geometry::Vector2D position, units::PolarAngle rotation);

	/**
	 * @brief Construct a new Linegular object.
	 *
	 * @param x The right-left position on the plane.
	 * @param y The forward-backward position on the plane.
	 * @param rotation The heading angle.
	 */
	Linegular(double x, double y, units::PolarAngle rotation);

	geometry::Vector2D getPosition();
	double getX();
	double getY();
	units::PolarAngle getRotation();

	double getXYMagnitude();

	void setPosition(geometry::Vector2D newPosition);
	void setPosition(double x, double y);
	void setRotation(units::PolarAngle newRotation);

	void rotateXYBy(units::PolarAngle rotation);
	void rotateExponentialBy(units::PolarAngle rotation);

	Linegular operator+(Linegular other);
	Linegular operator-(Linegular other);
	Linegular &operator+=(Linegular other);
	Linegular &operator-=(Linegular other);

	Linegular operator*(double value);
	Linegular operator/(double value);


	geometry::Vector2D position;
	units::PolarAngle rotation;
};


}
}
