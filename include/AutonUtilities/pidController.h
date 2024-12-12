#pragma once

#include "vex.h"

class PIDController {
public:
	PIDController(double kP = 0, double kI = 0, double kD = 0, double settleRange = 5, double settleFrameCount = 7);
	void resetErrorToZero();
	void computeFromError(double error);
	void setErrorI(double errorI);
	double getValue(bool useP = true, bool useI = true, bool useD = true);
	bool isSettled();

private:
	timer pidTimer;

	double kProp, kInteg, kDeriv;
	double currentError, cumulativeError, deltaError, previousError;
	double settleErrorRange, settleMinFrameCount;
	double settledFrames;
};
