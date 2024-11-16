#pragma once

namespace botintake {
	void runThread();

	void preauton();

	void setIntakeVelocity(double);
	double getIntakeVelocity();

	void setState(int, double = 0);

	void switchMode();

	void setHookMode(int);

	void switchFilterColor();

	void setFilterColor(char *);

	void control(int, int);

	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}
