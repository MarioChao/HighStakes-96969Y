#pragma once

class PIDControl {
public:
	PIDControl(double kP = 0, double kI = 0, double kD = 0, double settleRange = 5, double settleFrameCount = 7);
	void computeFromError(double error);
	void setErrorI(double errorI);
	double getValue(bool useP = true, bool useI = true, bool useD = true);
	bool isSettled();
private:
	double kProp, kInteg, kDeriv;
	double currentError, cumulativeError, deltaError, previousError;
	double settleErrorRange, settleMinFrameCount;
	double settledFrames;
};
