#pragma once

namespace botintake {
	void runThread();
	
	void preauton();
	
	void setState(int, double = 0);

	void setState2(int, double = 0);

	void setState3(int, double = 0);

	void switchMode();

	void switchFilterColor();

	void setFilterColor(char *);
	
	void control(int, int);
	
	bool canControl();

	extern int _taskState;
	extern double _taskDelay;
}