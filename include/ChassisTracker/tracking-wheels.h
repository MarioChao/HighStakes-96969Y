#pragma once

#include "main.h"

namespace chassis_tracker {
	class TrackingWheels;
}

/**
 * @param polarAngle The sensor's measuring direction, in degrees, on the 2D plane, with x-axis = 0 and counter-clockwise being positive.
 * @param revolutionCallback A function pointer for getting the sensor's immediate value in revolutions.
 * @param sensorToWheel_gearRatio The ratio multiplied to convert sensor revs to wheel revs, e.g. `sensorGearTeeth` / `wheelGearTeeth`.
 * @param wheelDiameter_inches The diameter of the driven wheel in inches.
 * @param normalRotateRadius_inches The distance between the sensor's measuring line and a parallel line passing through the tracking center. Positive means the sensor is measuring forward to the right of the tracking center.
 */
class chassis_tracker::TrackingWheels {
public:
	TrackingWheels();

private:
	double measureAngle_polarDegrees;
};
