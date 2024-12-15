#pragma once

namespace botarm {
	void runThread();

	void preauton();

	void setTargetAngle(double, double = 0);

	void setArmStage(int stageId, double delaySec = 0);
	int getArmStage();
	void resetArmEncoder();
	bool isArmResetted();
	void setResetDefaultStage(int stageId);

	void switchState();

	void control(int);

	bool canControl();

	extern double _taskState;
	extern double _taskDelay;
}
