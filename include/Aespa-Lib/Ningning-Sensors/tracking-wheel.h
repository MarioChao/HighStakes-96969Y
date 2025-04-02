#pragma once

#include "Aespa-Lib/Ningning-Sensors/encoder-beats.hpp"
#include "Aespa-Lib/Winter-Utilities/units.h"


namespace aespa_lib {
namespace sensor_beats {

class TrackingWheel {
public:
	/**
	 * @param encoder An encoder object that stores a sensor.
	 * @param directionAngle The sensor's 2D direction (polar angle), with x-axis = 0 and counter-clockwise being positive.
	 * @param sensorToWheel_gearRatio The ratio multiplied to sensor readings to obtain wheel readings.
	 * @param wheelDiameter_inches The diameter of the driven wheel in inches.
	 * @param offset_inches The distance between the sensor's measuring line and a parallel line passing through the tracking center. Positive means the sensor is measuring forward to the right of the tracking center.
	 */
	TrackingWheel(
		Encoder &sensor_encoder, units::PolarAngle directionAngle,
		double sensorToWheel_gearRatio, double wheelDiameter_inches, double offset_inches
	);

	double getDistance_inches();
	void storeDistance();
	double getRawDeltaDistance_inches(bool willStoreDistance = true);
	double getCenterDeltaDistance_inches(double deltaDistance_inches, double rotatedAngle_polarDegrees);

	units::PolarAngle getDirection();

private:
	/* Configs */
	Encoder &sensor_encoder;
	units::PolarAngle directionAngle;
	double sensorToWheel_gearRatio;
	double wheelDiameter_inches;
	double offset_inches;

	/* Internal states */
	double storedDistance_inches;
};

}
}
