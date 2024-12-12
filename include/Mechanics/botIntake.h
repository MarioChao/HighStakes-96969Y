#pragma once

#include <string>

namespace botintake {
	void runThread();

	void preauton();

	void setIntakeVelocity(double);
	double getIntakeVelocity();

	void setState(int, double = 0);

	/* Color filter */

	bool isColorFiltering();
	void setColorFiltering(bool, double = 0);

	void switchFilterColor();

	void setFilterOutColor(std::string);

	void setIntakeStoreRing(bool, double = 0);

	/* Control */

	void control(int, int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;

	extern bool _colorFilterTaskState;
	extern double _colorFilterTaskDelay;

	extern bool _storeRingTaskState;
	extern double _storeRingTaskDelay;
}
