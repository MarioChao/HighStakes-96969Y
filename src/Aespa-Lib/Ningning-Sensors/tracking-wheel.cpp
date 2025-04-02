#include "Aespa-Lib/Ningning-Sensors/tracking-wheel.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace aespa_lib {
namespace sensor_beats {

TrackingWheel::TrackingWheel(
	Encoder &sensor_encoder, units::PolarAngle directionAngle,
	double sensorToWheel_gearRatio, double wheelDiameter_inches, double offset_inches
)
	: sensor_encoder(sensor_encoder),
	directionAngle(directionAngle),
	sensorToWheel_gearRatio(sensorToWheel_gearRatio),
	wheelDiameter_inches(wheelDiameter_inches),
	offset_inches(offset_inches) {
}

double TrackingWheel::getDistance_inches() {
	double wheelAngle_degrees = sensor_encoder.getAngle_degrees() * sensorToWheel_gearRatio;
	double wheelDistance_inches = (wheelAngle_degrees / 360.0) * (M_PI * wheelDiameter_inches);
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
	auto arcRadiusResult = aespa_lib::genutil::getArcRadius_inches(deltaDistance_inches, rotatedAngle_polarDegrees);
	if (!arcRadiusResult.first) {
		// No turning
		return deltaDistance_inches;
	}

	// Turning
	double wheelArcRadius_inches = arcRadiusResult.second;
	double centerArcRadius_inches = wheelArcRadius_inches - offset_inches;
	double centerChordLength_inches = aespa_lib::genutil::getChordLength_inches(centerArcRadius_inches, rotatedAngle_polarDegrees);
	return centerChordLength_inches;
}

units::PolarAngle TrackingWheel::getDirection() {
	return directionAngle;
}

}
}
