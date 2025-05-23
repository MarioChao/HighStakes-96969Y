#pragma once

#include <string>

namespace botintake {
	void runThread();

	void preauton();

	void setIntakeVelocity(double);
	double getIntakeVelocity();

	void setState(int, double = 0);

	/* Torque / jam control */

	void setAntiJam(bool antiJamState);
	void setMaxTorque(double maxTorque_pct);


	/* Color filter */

	bool isColorFiltering();
	void setColorFiltering(bool, double = 0);

	void switchFilterColor();

	void setFilterOutColor(std::string colorText);
	std::string getFilterOutColor();

	void setIntakeStoreRing(bool, double = 0);


	/* Control */

	void control(int, int);

	bool canControl();
	void setControlState(bool canControl);

	extern int _taskState;
	extern double _taskDelay;

	extern bool _colorFilterTaskState;
	extern double _colorFilterTaskDelay;

	extern bool _storeRingTaskState;
	extern double _storeRingTaskDelay;
}
