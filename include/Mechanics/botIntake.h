#pragma once

namespace botintake {
	void runThread();

	void preauton();

	void setIntakeVelocity(double);
	double getIntakeVelocity();

	void setState(int, double = 0);

	bool isColorFiltering();
	void setColorFiltering(bool isEnabled);

	void switchFilterColor();

	void setFilterColor(char *);

	void control(int, int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
