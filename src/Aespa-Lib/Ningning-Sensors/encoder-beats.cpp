#include "Aespa-Lib/Ningning-Sensors/encoder-beats.hpp"

namespace aespa_lib {
namespace sensor_beats {

// ---------- Encoder ----------

double Encoder::getAngle_degrees() {
	return 0;
};

void Encoder::setAngle_degrees(double angle_degrees) {
	// getAngle() + angleOffset = realAngle
	// angleOffset = realAngle - getAngle()
	angleOffset_degrees = angle_degrees - getAngle_degrees();
};

// ---------- Rotation Sensor ----------

RotationSensor::RotationSensor(rotation &sensor)
	: sensor(sensor) {
}

double RotationSensor::getAngle_degrees() {
	return sensor.position(degrees);
}

// ---------- Optical Shaft Encoder ----------

OpticalShaftEncoder::OpticalShaftEncoder(encoder &sensor)
	: sensor(sensor) {
}

double OpticalShaftEncoder::getAngle_degrees() {
	return sensor.position(degrees);
}

// ---------- Motor ----------

Motor::Motor(motor &sensor)
	: sensor(sensor) {
}

double Motor::getAngle_degrees() {
	return sensor.position(degrees);
}

}
}
