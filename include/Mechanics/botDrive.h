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

	void setMaxDriveVelocity(double velocityPct);
	double getMaxDriveVelocity();

	void driveLinegularVelocity(double linearVelocity_pct, double angularVelocity_radPerSecond);
	void driveVelocity(double leftVelocityPct, double rightVelocityPct);
	void driveVoltage(double leftVoltageVolt, double rightVoltageVolt, double clampMaxVoltage);
}
