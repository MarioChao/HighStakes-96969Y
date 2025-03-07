#pragma once

namespace pas1_lib {
namespace auton {
namespace control_loops {

class ForwardController {
public:
	ForwardController(double kS = 0, double kV = 0, double kA = 0);

	void computeFromMotion(double velocity, double acceleration);
	double getValue(bool useS = true, bool useV = true, bool useA = true);


	double kVelo, kAcel, kStat;

private:
	double velocity, acceleration;
};

}
}
}
