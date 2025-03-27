#include "Pas1-Lib/Basic-Control/Chassis/differential.h"

namespace pas1_lib {
namespace basic_control {
namespace chassis {

Differential::Differential(
	chassis_tracker::Odometry &odometry, AutonSettings &autonSettings,
	motor_group &left_motors, motor_group &right_motors
)
	: ChassisBase(odometry, autonSettings),
	left_motors(left_motors), right_motors(right_motors) {}

void Differential::control_local2d(
	double right_pct, double look_pct,
	double angular_pct
) {
	// Linear + angular
	double leftMotor_pct = look_pct - angular_pct;
	double rightMotor_pct = look_pct + angular_pct;

	// Scale if pct >= 100
	double pctScaleFactor = aespa_lib::genutil::getScaleFactor(
		100, { leftMotor_pct, rightMotor_pct }
	);
	leftMotor_pct *= pctScaleFactor;
	rightMotor_pct *= pctScaleFactor;

	// Convert to volt
	double leftMotor_volt = aespa_lib::genutil::pctToVolt(leftMotor_pct);
	double rightMotor_volt = aespa_lib::genutil::pctToVolt(rightMotor_pct);

	// Command
	left_motors.spin(fwd, leftMotor_volt, volt);
	right_motors.spin(fwd, rightMotor_volt, volt);
}

void Differential::stopMotors(brakeType mode) {
	left_motors.stop(mode);
	right_motors.stop(mode);
}

}
}
}
