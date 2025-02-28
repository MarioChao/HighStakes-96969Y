#include "ChassisTracker/tracking-wheel.h"
#include "Utilities/generalUtility.h"

namespace chassis_tracker {
	TrackingWheel::TrackingWheel(
		sensor_beats::Encoder &sensor_encoder, double directionAngle_polarDegrees,
		double sensorToWheel_gearRatio, double wheelDiameter_inches, double offset_inches
	)
	: sensor_encoder(sensor_encoder),
	directionAngle_polarDegrees(directionAngle_polarDegrees),
	sensorToWheel_gearRatio(sensorToWheel_gearRatio),
	wheelDiameter_inches(wheelDiameter_inches),
	offset_inches(offset_inches)
	{
		storeDistance();
	}

	double TrackingWheel::getDistance_inches() {
		double wheelAngle_degrees = sensor_encoder.getAngle_degrees() * sensorToWheel_gearRatio;
		double wheelDistance_inches = wheelAngle_degrees * (M_PI * wheelDiameter_inches);
		return wheelDistance_inches;
	}

	void TrackingWheel::storeDistance() {
		storedDistance_inches = getDistance_inches();
	}

	double TrackingWheel::getRawDeltaDistance_inches(bool willStoreDistance) {
		double currentDistance_inches = getDistance_inches();
		double deltaDistance_inches = currentDistance_inches - storedDistance_inches;
		if (willStoreDistance) {
			storedDistance_inches = currentDistance_inches;
		}
		return deltaDistance_inches;
	}

	double TrackingWheel::getCenterDeltaDistance_inches(double deltaDistance_inches, double rotatedAngle_polarDegrees) {
		auto arcRadiusResult = genutil::getArcRadius_inches(deltaDistance_inches, rotatedAngle_polarDegrees);
		if (!arcRadiusResult.first) {
			// No turning
			return deltaDistance_inches;
		}
		
		// Turning
		double wheelArcRadius_inches = arcRadiusResult.second;
		double centerArcRadius_inches = wheelArcRadius_inches - offset_inches;
		double centerChordLength_inches = genutil::getChordLength_inches(centerArcRadius_inches, rotatedAngle_polarDegrees);
		return centerChordLength_inches;
	}

	double TrackingWheel::getDirection_polarDegrees() {
		return directionAngle_polarDegrees;
	}
}
