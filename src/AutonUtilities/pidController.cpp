#include "AutonUtilities/pidController.h"
#include "main.h"

PIDController::PIDController(double kP, double kI, double kD, double settleRange, double settleFrameCount) {
	kProp = kP, kInteg = kI, kDeriv = kD;
	resetErrorToZero();

	settleErrorRange = fabs(settleRange);
	settleMinFrameCount = settleFrameCount;
	settledFrames = 0;
}

void PIDController::resetErrorToZero() {
	previousError = currentError = 2e17;
	cumulativeError = deltaError = 0;
	pidTimer.reset();
	settledFrames = 0;
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
		cumulativeError = 0;
	} else {
		cumulativeError += 0.5 * (previousError + currentError) * elapsedTime_seconds;
	}
	deltaError = (currentError - previousError) / elapsedTime_seconds;

	// Settle errors
	if (fabs(error) < settleErrorRange) {
		settledFrames++;
		settledFrames = fmin(settledFrames, settleMinFrameCount + 1);
	} else {
		settledFrames = 0;
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
	if (fabs(currentError) < settleErrorRange && settledFrames >= settleMinFrameCount) {
		return true;
	} else {
		return false;
	}
}
