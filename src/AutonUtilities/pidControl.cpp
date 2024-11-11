#include "AutonUtilities/pidControl.h"
#include "main.h"

PIDControl::PIDControl(double kP, double kI, double kD, double settleRange, double settleFrameCount) {
	kProp = kP, kInteg = kI, kDeriv = kD;
	cumulativeError = deltaError = 0;
	currentError = previousError = 2e17;

	settleErrorRange = fabs(settleRange);
	settleMinFrameCount = settleFrameCount;
	settledFrames = 0;
}

void PIDControl::computeFromError(double error) {
	// Previous error
	if (previousError > 1e17) {
		previousError = error;
	} else {
		previousError = currentError;
	}

	// Calculate errors
	currentError = error;
	bool isCrossZero = (currentError >= 0 && previousError <= 0) || (currentError <= 0 && previousError >= 0);
	if (isCrossZero) {
		cumulativeError = 0;
	} else {
		cumulativeError += error;
	}
	deltaError = currentError - previousError;

	// Settle errors
	if (fabs(error) < settleErrorRange) {
		settledFrames++;
		settledFrames = fmin(settledFrames, settleMinFrameCount + 1);
	} else {
		settledFrames = 0;
	}
}

void PIDControl::setErrorI(double errorI) {
	cumulativeError = errorI;
}

double PIDControl::getValue(bool useP, bool useI, bool useD) {
	double valP = useP ? (currentError * kProp) : 0;
	double valI = useI ? (cumulativeError * kInteg) : 0;
	double valD = useD ? (deltaError * kDeriv) : 0;
	return valP + valI + valD;
}

bool PIDControl::isSettled() {
	if (fabs(currentError) < settleErrorRange && settledFrames >= settleMinFrameCount) {
		return true;
	} else {
		return false;
	}
}
