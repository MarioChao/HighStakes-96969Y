#pragma once

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
	double getTheta_degrees();
	double getTheta_radians();

	void rotateXYBy(double polarRotate_radians);

	Linegular operator+(Linegular &other);
	Linegular operator-(Linegular &other);

private:
	double x, y, theta;
};
