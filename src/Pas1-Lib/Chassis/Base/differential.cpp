#include "Pas1-Lib/Chassis/Base/differential.h"


namespace pas1_lib {
namespace chassis {
namespace base {

Differential::Differential(
	settings::Odometry &odometry, settings::BotInfo &botInfo, settings::AutonSettings &autonSettings,
	motor_group &left_motors, motor_group &right_motors
)
	: ChassisBase(odometry, botInfo, autonSettings),
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

double Differential::getLookVelocity() {
	double averageVelocity_rpm = (left_motors.velocity(rpm) + right_motors.velocity(rpm)) / 2.0;
	return averageVelocity_rpm * (1.0 / 60.0) * botInfo.wheelCircum_tiles * botInfo.motorToWheel_gearRatio;
}

}
}
}
