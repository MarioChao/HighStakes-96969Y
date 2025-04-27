#include "Pas1-Lib/Chassis/Base/differential.h"


namespace pas1_lib {
namespace chassis {
namespace base {

Differential::Differential(
	settings::Odometry &odometry, settings::BotInfo &botInfo, settings::AutonSettings &autonSettings,
	motor_group &left_motors, motor_group &right_motors
)
	: ChassisBase(odometry, botInfo, autonSettings),
	left_motors(left_motors), right_motors(right_motors),
	leftAcceleration_pctPerSec_slew(autonSettings.motorAcceleration_pctPerSec_slew),
	rightAcceleration_pctPerSec_slew(autonSettings.motorAcceleration_pctPerSec_slew) {
	overwriteLeftRightVelocity = { false, {0, 0} };
}

void Differential::control_differential(double left_pct, double right_pct, bool useSlew) {
	if (useSlew) {
		// Slew velocity
		leftAcceleration_pctPerSec_slew.computeFromTarget(left_pct);
		rightAcceleration_pctPerSec_slew.computeFromTarget(right_pct);

		// Store velocity
		desired_leftMotor_pct = leftAcceleration_pctPerSec_slew.getValue();
		desired_rightMotor_pct = rightAcceleration_pctPerSec_slew.getValue();
	} else {
		desired_leftMotor_pct = left_pct;
		desired_rightMotor_pct = right_pct;
	}


	commanded_leftMotor_volt = aespa_lib::genutil::pctToVolt(desired_leftMotor_pct);
	commanded_rightMotor_volt = aespa_lib::genutil::pctToVolt(desired_rightMotor_pct);

	double scaleFactor = aespa_lib::genutil::getScaleFactor(12, { commanded_leftMotor_volt, commanded_rightMotor_volt });
	commanded_leftMotor_volt *= scaleFactor;
	commanded_rightMotor_volt *= scaleFactor;
	// commanded_leftMotor_volt = aespa_lib::genutil::clamp(commanded_leftMotor_volt, -12, 12);
	// commanded_rightMotor_volt = aespa_lib::genutil::clamp(commanded_rightMotor_volt, -12, 12);

	// Command
	left_motors.spin(fwd, commanded_leftMotor_volt, volt);
	right_motors.spin(fwd, commanded_rightMotor_volt, volt);
}

void Differential::control_local2d(
	double right_pct, double look_pct,
	double angular_pct,
	bool useSlew
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

	// printf("%.3f %.3f %.3f %.3f\n", look_pct, angular_pct, tempLeftMotor_pct, tempRightMotor_pct);

	// Control
	control_differential(tempLeftMotor_pct, tempRightMotor_pct, useSlew);
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

double Differential::getLeftVelocity() {
	if (overwriteLeftRightVelocity.first) return overwriteLeftRightVelocity.second.first;

	return left_motors.velocity(rpm) * (1.0 / 60.0) * botInfo.wheelCircum_tiles * botInfo.motorToWheel_gearRatio;
}

double Differential::getRightVelocity() {
	if (overwriteLeftRightVelocity.first) return overwriteLeftRightVelocity.second.second;

	return right_motors.velocity(rpm) * (1.0 / 60.0) * botInfo.wheelCircum_tiles * botInfo.motorToWheel_gearRatio;
}

}
}
}
