#pragma once

class ForwardController {
public:
	ForwardController(double kS = 0, double kV = 0, double kA = 0);
	void computeFromMotion(double velocity, double acceleration);
	double getValue(bool useS = true, bool useV = true, bool useA = true);

private:
	double kVelo, kAcel, kStat;
	double velocity, acceleration;
};
