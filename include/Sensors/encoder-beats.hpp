#pragma once

#include "main.h"

namespace sensor_beats {
	class Encoder;
	class RotationSensor;
	class OpticalShaftEncoder;
	class Motor;
}

class sensor_beats::Encoder {
public:
	virtual double getAngle_degrees();
	void setAngle_degrees(double angle_degrees);

private:
	double angleOffset_degrees;
};

class sensor_beats::RotationSensor : public sensor_beats::Encoder {
public:
	RotationSensor(rotation &sensor);
	double getAngle_degrees() override;

private:
	rotation &sensor;
};

class sensor_beats::OpticalShaftEncoder : public sensor_beats::Encoder {
public:
	OpticalShaftEncoder(encoder &sensor);
	double getAngle_degrees() override;

private:
	encoder &sensor;
};

class sensor_beats::Motor : public sensor_beats::Encoder {
public:
	Motor(motor &sensor);
	double getAngle_degrees() override;

private:
	motor &sensor;
};
