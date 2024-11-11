#pragma once

namespace odometry {
	void startThreads();

	double getX();
	double getY();
	double getAngle();

	void setValues(double, double, double);
}
