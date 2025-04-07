#pragma once

namespace pas1_lib {
namespace auton {
namespace control_loops {


class SimpleFeedforward {
public:
	SimpleFeedforward(double kS, double kV, double kA = 0, double period_sec = 0.020);

	double calculateDiscrete(double currentVelocity, double nextVelocity);
	void computeFromMotion(double velocity, double acceleration);
	double getValue(bool useS = true, bool useV = true, bool useA = true);


	double kS, kV, kA;
	double period_sec;

private:
	double velocity, acceleration;
};


class ArmFeedforward {
public:
	ArmFeedforward(double kS, double kG, double kV, double kA = 0, double period_sec = 0.020);

	// double calculateDiscrete(double currentAngle_rad, double currentVelocity, double nextVelocity);
	double calculateFromMotion(double elevationAngle_rad, double angularVelocity, double angularAcceleration);


	double kS, kG, kV, kA;
	double period_sec;
};


}
}
}
