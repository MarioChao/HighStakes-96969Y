#pragma once

#include "Pas1-Lib/Chassis/Base/chassis-base.h"


namespace pas1_lib {
namespace chassis {
namespace base {

class Differential : public ChassisBase {
public:
	Differential(
		settings::Odometry &odometry, settings::BotInfo &botInfo, settings::AutonSettings &autonSettings,
		motor_group &left_motors, motor_group &right_motors
	);

	void control_differential(double left_pct, double right_pct);
	void control_local2d(
		double right_pct, double look_pct,
		double angular_pct
	) override;
	void stopMotors(brakeType mode) override;

	double getLookVelocity() override;
	double getAngularVelocity() override;

	double getLeftVelocity();
	double getRightVelocity();


	std::pair<bool, std::pair<double, double>> overwriteLeftRightVelocity;

	double desired_leftMotor_pct, desired_rightMotor_pct;
	double commanded_leftMotor_volt, commanded_rightMotor_volt;

private:
	motor_group &left_motors, &right_motors;
};

}
}
}
