#pragma once

namespace aespa_lib {
namespace datas {

struct Vector2D {
	Vector2D(double x, double y);

	void rotateBy(double polarRotate_radians);
	void rotateExponentialBy(double polarRotate_radians);

	double getMagnitude();

	double x, y;
};

// Class containing 2D position & rotation data
class Linegular {
public:
	Linegular(Vector2D position, double polarTheta_degrees);

	/**
	 * @brief Construct a new Linegular object.
	 *
	 * @param x The right-left position on the plane.
	 * @param y The forward-backward position on the plane.
	 * @param polarTheta_degrees The polar rotation in degrees.
	 */
	Linegular(double x, double y, double polarTheta_degrees);

	Vector2D getPosition();
	double getX();
	double getY();
	double getThetaPolarAngle_degrees();
	double getThetaPolarAngle_radians();

	double getXYMagnitude();

	void setPosition(Vector2D newPosition);
	void setPosition(double x, double y);

	void rotateXYBy(double polarRotate_radians);
	void rotateExponentialBy(double polarRotate_radians);

	Linegular operator+(Linegular &other);
	Linegular operator-(Linegular &other);

	Linegular operator*(double value);
	Linegular operator/(double value);

private:
	Vector2D position;
	double theta_degrees;
};

}
}
