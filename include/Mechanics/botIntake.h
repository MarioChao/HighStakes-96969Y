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
	void setColorFiltering(bool isEnabled);

	void switchFilterColor();

	void setFilterOutColor(std::string);

	void setIntakeStoreRing(bool isStore);

	/* Control */

	void control(int, int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
