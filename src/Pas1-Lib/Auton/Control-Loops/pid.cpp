#include "Pas1-Lib/Auton/Control-Loops/pid.h"

#include "Aespa-Lib/Winter-Utilities/general.h"


namespace pas1_lib {
namespace auton {
namespace control_loops {

PIDController::PIDController(double kP, PID_kI_params kI, double kD, std::vector<end_conditions::Settle> settleControllers)
	: kProp(kP), kI_params(kI), kDeriv(kD),
	settleControllers(settleControllers) {
	resetErrorToZero();
}

PIDController::PIDController(double kP, PID_kI_params kI, double kD, double settleRange, double settleFrameCount)
	: PIDController(kP, kI, kD, { end_conditions::Settle(settleRange, settleFrameCount) }) {}

void PIDController::resetErrorToZero() {
	previousError = currentError = 2e17;
	cumulativeError = deltaError = 0;
	pidTimer.reset();

	for (end_conditions::Settle &settleControl : settleControllers) {
		settleControl.reset();
	}
}

void PIDController::computeFromError(double error) {
	// Previous error
	if (previousError > 1e17) {
		previousError = error;
	} else {
		previousError = currentError;
	}

	// Elapsed time
	double elapsedTime_seconds = pidTimer.value();
	pidTimer.reset();


	/* ---------- Update errors ---------- */

	// Proportional
	currentError = error;

	// Integral
	setErrorI(cumulativeError + 0.5 * (previousError + error) * elapsedTime_seconds);

	// I reset
	bool isCrossZero = aespa_lib::genutil::signum(error) != aespa_lib::genutil::signum(previousError);
	if (kI_params.signFlipReset && isCrossZero) setErrorI(0);
	else if (kI_params.windUpRange > 0 && error > kI_params.windUpRange) setErrorI(0);

	// Derivative
	if (elapsedTime_seconds > 1e-5) {
		deltaError = (error - previousError) / elapsedTime_seconds;
	} else {
		deltaError = 0;
	}

	// Settle controllers
	for (end_conditions::Settle &settleControl : settleControllers) {
		settleControl.computeFromError(error);
	}
}

void PIDController::setErrorI(double errorI) {
	cumulativeError = errorI;
}

double PIDController::getValue(bool useP, bool useI, bool useD) {
	double valP = useP ? (currentError * kProp) : 0;
	double valI = useI ? (cumulativeError * kI_params.kI) : 0;
	double valD = useD ? (deltaError * kDeriv) : 0;
	return valP + valI + valD;
}

bool PIDController::isSettled() {
	bool isSettled_state = false;

	// Check if any controllers settled
	for (end_conditions::Settle &settleControl : settleControllers) {
		if (settleControl.isSettled()) {
			isSettled_state = true;
			break;
		}
	}

	return isSettled_state;
}

}
}
}
