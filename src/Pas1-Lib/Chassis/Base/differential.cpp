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
	double tempLeftMotor_pct = look_pct - angular_pct;
	double tempRightMotor_pct = look_pct + angular_pct;

	// Scale if pct >= 100
	double pctScaleFactor = aespa_lib::genutil::getScaleFactor(
		100, { tempLeftMotor_pct, tempRightMotor_pct }
	);
	tempLeftMotor_pct *= pctScaleFactor;
	tempRightMotor_pct *= pctScaleFactor;

	// Store velocity pct
	desired_leftMotor_pct = tempLeftMotor_pct;
	desired_rightMotor_pct = tempRightMotor_pct;

	// Get volt
	// double currentLeft_pct = left_motors.velocity(pct);
	// double currentRight_pct = right_motors.velocity(pct);
	// double leftMotor_volt = left_motors.voltage(volt);
	// double rightMotor_volt = right_motors.voltage(volt);
	// autonSettings.velocityError_pct_to_volt_pid.computeFromError(desired_leftMotor_pct - currentLeft_pct);
	// leftMotor_volt += autonSettings.velocityError_pct_to_volt_pid.getValue();
	// autonSettings.velocityError_pct_to_volt_pid.computeFromError(desired_rightMotor_pct - currentRight_pct);
	// rightMotor_volt += autonSettings.velocityError_pct_to_volt_pid.getValue();

	// commanded_leftMotor_volt = leftMotor_volt;
	// commanded_rightMotor_volt = rightMotor_volt;

	commanded_leftMotor_volt = aespa_lib::genutil::pctToVolt(desired_leftMotor_pct);
	commanded_rightMotor_volt = aespa_lib::genutil::pctToVolt(desired_rightMotor_pct);

	commanded_leftMotor_volt = aespa_lib::genutil::clamp(commanded_leftMotor_volt, -12, 12);
	commanded_rightMotor_volt = aespa_lib::genutil::clamp(commanded_rightMotor_volt, -12, 12);
	
	// Command
	// left_motors.spin(fwd, desired_leftMotor_pct, velocityUnits::pct);
	// right_motors.spin(fwd, desired_rightMotor_pct, velocityUnits::pct);
	left_motors.spin(fwd, commanded_leftMotor_volt, volt);
	right_motors.spin(fwd, commanded_rightMotor_volt, volt);
}

void Differential::stopMotors(brakeType mode) {
	desired_leftMotor_pct = desired_rightMotor_pct = 0;
	commanded_leftMotor_volt = commanded_rightMotor_volt = 0;
	left_motors.stop(mode);
	right_motors.stop(mode);
}

double Differential::getLookVelocity() {
	if (overwriteLookVelocity.first) return overwriteLookVelocity.second;

	double averageVelocity_rpm = (left_motors.velocity(rpm) + right_motors.velocity(rpm)) / 2.0;
	return averageVelocity_rpm * (1.0 / 60.0) * botInfo.wheelCircum_tiles * botInfo.motorToWheel_gearRatio;
}

double Differential::getAngularVelocity() {
	double averageVelocity_rpm = (right_motors.velocity(rpm) - left_motors.velocity(rpm)) / 2.0;
	return averageVelocity_rpm * (1.0 / 60.0) * botInfo.wheelCircum_tiles * botInfo.motorToWheel_gearRatio / (botInfo.trackWidth_tiles / 2);
}

}
}
}
