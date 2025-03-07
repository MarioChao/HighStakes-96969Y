#pragma once

#include "vex.h"

namespace aespa_lib {
namespace sensor_beats {

// List
class Encoder;
class RotationSensor;
class OpticalShaftEncoder;
class Motor;

// Declaration

class Encoder {
public:
	virtual double getAngle_degrees();
	void setAngle_degrees(double angle_degrees);

private:
	double angleOffset_degrees;
};

class RotationSensor : public Encoder {
public:
	RotationSensor(rotation &sensor);
	double getAngle_degrees() override;

private:
	rotation &sensor;
};

class OpticalShaftEncoder : public Encoder {
public:
	OpticalShaftEncoder(encoder &sensor);
	double getAngle_degrees() override;

private:
	encoder &sensor;
};

class Motor : public Encoder {
public:
	Motor(motor &sensor);
	double getAngle_degrees() override;

private:
	motor &sensor;
};

}
}
