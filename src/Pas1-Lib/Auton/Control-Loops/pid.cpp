#include "Pas1-Lib/Auton/Control-Loops/pid.h"

namespace pas1_lib {
namespace auton {
namespace control_loops {

PIDController::PIDController(double kP, double kI, double kD, std::vector<end_conditions::Settle> settleControllers)
	: kProp(kP), kInteg(kI), kDeriv(kD),
	settleControllers(settleControllers) {
	resetErrorToZero();
}

PIDController::PIDController(double kP, double kI, double kD, double settleRange, double settleFrameCount)
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

	// Update errors
	currentError = error;
	bool isCrossZero = (currentError >= 0 && previousError <= 0) || (currentError <= 0 && previousError >= 0);
	if (isCrossZero) {
		setErrorI(0);
	} else {
		setErrorI(cumulativeError + 0.5 * (previousError + currentError) * elapsedTime_seconds);
	}
	if (elapsedTime_seconds > 1e-5) {
		deltaError = (currentError - previousError) / elapsedTime_seconds;
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
	double valI = useI ? (cumulativeError * kInteg) : 0;
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
