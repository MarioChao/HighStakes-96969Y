#pragma once

class Linegular {
public:
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
