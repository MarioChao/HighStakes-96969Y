#pragma once

namespace botdrive {
	enum controlType {
		ArcadeTwoStick,
		ArcadeSingleStick,
	};

	void runThread();

	void preauton();

	void switchDriveMode();

	void control();

	bool canControl();
	void setControlState(bool canControl);

	void setMaxDriveVelocity(double velocityPct);
	double getMaxDriveVelocity();

	void setMaxDeltaVolt(double deltaVolt);

	void driveLinegularVelocity(double linearVelocity_pct, double angularVelocity_radPerSecond);
	void driveVelocity(double leftVelocityPct, double rightVelocityPct);
	void driveVoltage(double leftVoltageVolt, double rightVoltageVolt, double clampMaxVoltage);
}
