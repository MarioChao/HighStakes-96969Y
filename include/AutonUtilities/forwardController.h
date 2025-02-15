#pragma once

class ForwardController {
public:
	ForwardController(double kV = 0, double kA = 0, double kS = 0);
	void computeFromMotion(double velocity, double acceleration);
	double getValue(bool useV = true, bool useA = true, bool useS = true);

private:
	double kVelo, kAcel, kStat;
	double velocity, acceleration;
};
