#pragma once

class PatienceController {
public:
	PatienceController(int maxPatience, double minDelta, bool positiveImprovement = true, int delayComputePatience = 15);

	void reset();
	void computePatience(double value);

	void exhaustNow();
	bool isExhausted();

	void printDebug();

private:
	int maxPatienceLevel;
	double absoluteMinDelta;

	double storedValue;
	int patience;

	int delayComputePatienceLevel;
	int delayedCalls;

	bool positiveImprovement;
};
