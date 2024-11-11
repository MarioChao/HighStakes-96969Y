#include "AutonUtilities/odometry-old.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "main.h"

namespace {
	void odometryThread();

	double x, y, angle;
}

namespace odometry {
	void startThreads() {
		task odometryTask([]() -> int { odometryThread(); return 1; });
	}

	double getX() {
		return x;
	}

	double getY() {
		return y;
	}

	double getAngle() {
		return angle;
	}

	/// @brief Set the values of odometry
	/// @param x Horizontal position in tiles
	/// @param y Vertical position in tiles
	/// @param angle Clockwise angle in degrees
	void setValues(double x, double y, double angle) {
		::x = x;
		::y = y;
		::angle = angle;
	}
}

namespace {
	void odometryThread() {
		// Variables
		double oldLook = LookRotation.position(rev);
		double oldRight = RightRotation.position(rev);
		double oldAngle = InertialSensor.rotation();

		// Loop
		while (true) {
			// Get new sensor values
			double newLook = LookRotation.position(rev);
			double newRight = RightRotation.position(rev);
			double newAngle = InertialSensor.rotation();

			// Get local difference
			double localDeltaLook = newLook - oldLook;
			double localDeltaRight = newRight - oldRight;
			double deltaAngle = newAngle - oldAngle;

			// Rotate to absolute difference
			double averageAngle = oldAngle + deltaAngle / 2;
			double rotateAngleRad = averageAngle * M_PI / 180.0;
			double absoluteDeltaLook = localDeltaLook * cos(rotateAngleRad) - localDeltaRight * sin(rotateAngleRad);
			double absoluteDeltaRight = localDeltaLook * sin(rotateAngleRad) + localDeltaRight * cos(rotateAngleRad);

			// Update odometry values
			double revToTiles = (1.0 / botinfo::trackingLookWheelSensorGearRatio) * (botinfo::trackingLookWheelCircumIn / 1.0) * (1.0 / field::tileLengthIn);
			x += absoluteDeltaRight * revToTiles;
			y += absoluteDeltaLook * revToTiles;
			angle = newAngle;

			// Update old sensor values
			oldLook = newLook;
			oldRight = newRight;
			oldAngle = newAngle;

			// Delay for while loop
			task::sleep(20);
		}
	}
}
