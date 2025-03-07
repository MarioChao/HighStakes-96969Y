#pragma once

#include "vex.h"
#include "Pas1-Lib/Auton/End-Conditions/settle.h"
#include <vector>


namespace pas1_lib {
namespace auton {
namespace control_loops {

class PIDController {
public:
	PIDController(double kP, double kI, double kD, std::vector<end_conditions::Settle> settleControllers);
	PIDController(double kP = 0, double kI = 0, double kD = 0, double settleRange = 5, double settleTime_seconds = 0.1);

	void resetErrorToZero();
	void computeFromError(double error);

	void setErrorI(double errorI);
	double getValue(bool useP = true, bool useI = true, bool useD = true);

	bool isSettled();


	double kProp, kInteg, kDeriv;

private:
	timer pidTimer;

	double currentError, cumulativeError, deltaError, previousError;

	std::vector<pas1_lib::auton::end_conditions::Settle> settleControllers;
};

}
}
}
