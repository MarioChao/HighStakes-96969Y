#pragma once

#include "Pas1-Lib/Basic-Control/Chassis/chassis-base.h"


namespace pas1_lib {
namespace basic_control {
namespace chassis {

class Differential : public ChassisBase {
public:
	Differential(
		chassis_tracker::Odometry &odometry, BotInfo &botInfo, AutonSettings &autonSettings,
		motor_group &left_motors, motor_group &right_motors
	);

	void control_local2d(
		double right_pct, double look_pct,
		double angular_pct
	) override;
	void stopMotors(brakeType mode) override;

	double getLookVelocity() override;

private:
	motor_group &left_motors, &right_motors;
};

}
}
}
