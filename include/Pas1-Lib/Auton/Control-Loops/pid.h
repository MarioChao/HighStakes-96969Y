#pragma once

#include "vex.h"
#include "Pas1-Lib/Auton/End-Conditions/settle.h"
#include <vector>


namespace pas1_lib {
namespace auton {
namespace control_loops {


struct PID_kI_params {
	PID_kI_params(double kI = 0, double windUpRange = -1, bool signFlipReset = false)
		: kI(kI), windUpRange(windUpRange), signFlipReset(signFlipReset) {}


	double kI = 0;
	double windUpRange = -1;
	bool signFlipReset = false;
};


class PIDController {
public:
	PIDController(double kP, PID_kI_params kI_params, double kD, std::vector<end_conditions::Settle> settleControllers);
	PIDController(double kP = 0, PID_kI_params kI = 0, double kD = 0, double settleRange = 5, double settleTime_seconds = 0.1);

	void resetErrorToZero();
	void computeFromError(double error);

	void setErrorI(double errorI);
	double getValue(bool useP = true, bool useI = true, bool useD = true);

	bool isSettled();


	double kProp, kDeriv;
	PID_kI_params kI_params;

private:
	timer pidTimer;

	double currentError, cumulativeError, deltaError, previousError;

	std::vector<pas1_lib::auton::end_conditions::Settle> settleControllers;
};


}
}
}
