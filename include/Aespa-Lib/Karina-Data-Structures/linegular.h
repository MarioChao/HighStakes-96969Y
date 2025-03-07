#pragma once

namespace aespa_lib {
namespace datas {

// Class containing X, Y, and angle
class Linegular {
public:
	/**
	 * @brief Construct a new Linegular object.
	 *
	 * @param x The right-left position on the plane.
	 * @param y The forward-backward position on the plane.
	 * @param polarTheta_degrees The polar rotation in degrees.
	 */
	Linegular(double x, double y, double polarTheta_degrees);

	double getX();
	double getY();
	double getThetaPolarAngle_degrees();
	double getThetaPolarAngle_radians();

	double getXYMagnitude();

	void rotateXYBy(double polarRotate_radians);
	void rotateExponentialBy(double polarRotate_radians);

	Linegular operator+(Linegular &other);
	Linegular operator-(Linegular &other);

	Linegular operator*(double value);
	Linegular operator/(double value);

private:
	double x, y, theta_degrees;
};

}
}
