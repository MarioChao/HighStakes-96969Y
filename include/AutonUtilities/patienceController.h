#pragma once

class PatienceController {
public:
	PatienceController(int maxPatience, double minDelta, bool positiveImprovement = true);

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

	bool positiveImprovement;
};
