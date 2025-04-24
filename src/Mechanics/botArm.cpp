#include "Mechanics/botArm.h"

#include "Pas1-Lib/Auton/Control-Loops/feedforward.h"
#include "Pas1-Lib/Auton/Control-Loops/pid.h"
#include "Pas1-Lib/Auton/End-Conditions/patience.h"

#include "Aespa-Lib/Winter-Utilities/general.h"
#include "Aespa-Lib/Winter-Utilities/units.h"

#include "Utilities/debugFunctions.h"
#include "main.h"


namespace {
using pas1_lib::auton::control_loops::ArmFeedforward;
using pas1_lib::auton::control_loops::PIDController;
using pas1_lib::auton::end_conditions::PatienceController;

void resolveArmExtreme();
void resolveArmDegrees();
void resolveArmDirection();

bool isExtreme();

void setArmPosition(double armEncoderPosition_degrees);
void spinArmMotor(double velocityPct);

// Motor config
double armMaxVelocity_rpm = 100;
double armMaxVelocity_radiansPerSec = armMaxVelocity_rpm * (1.0 / 60.0) * (2 * M_PI);

double armEncoder_to_arm_ratio = 1.0 / 1.0;

/* Stage controllers */

// Arm feedforward
ArmFeedforward arm_velocity_radiansPerSec_to_volt_feedforward(0, 0.3, 0);
PIDController arm_positionError_radians_to_radiansPerSec_pid(0, 0, 0);
// PIDController arm_positionError_radians_to_radiansPerSec_pid(2.0, 0, 0);
// Pid
PIDController arm_positionError_radians_to_volt_pid_feedback(11, 0, 0.4);

// Patience
PatienceController armUpPatience(12, 1.0, true, 5);
PatienceController armDownPatience(6, 1.0, false, 5);

// Stage config
std::vector<double> armStages_degrees = { -2, 12, 25, 60, 115, 130, 180, 200, 230, 0};
// std::vector<double> armStages_degrees = { 0, 0, 25, 40, 180, 130, 180, 200, 210, 0 }; // angles for tuning
std::vector<int> extremeStages_values = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };
int currentArmStage = -1;
bool releaseOnExhausted = true;

// Reset arm info
bool armResetted = false;
int resetDefaultStageId = -1;

// Speed config
double defaultArmMaxVelocity_pct = 100;
double armMaxVelocity_pct = 100;
double armUpVelocityPct = 100;

// PID or direction
double armStateTargetAngle_degrees = 0;
int armStateDirection = 0;

bool useDirection = false;

bool controlState = true;
}


namespace botarm {
void runThread() {
	while (true) {
		// Thread code here
		if (useDirection) {
			resolveArmDirection();
		} else {
			// printf("st: %d, armvolt: %.3f\n", currentArmStage, ArmMotors.voltage(volt));
			// printf("arm torque: %.3f Nm\n", ArmMotors.torque());
			resolveArmDegrees();
		}

		wait(10, msec);
	}
}

void preauton() {
	ArmMotors.setPosition(0, degrees);
}

void setTargetAngle(double state, double delaySec) {
	// Check for instant set
	if (delaySec <= 1e-9) {
		// Set state here
		arm_positionError_radians_to_radiansPerSec_pid.setErrorI(0);
		armStateTargetAngle_degrees = state;

		return;
	}

	// Set global variables
	_taskState = state;
	_taskDelay = delaySec;

	task setTargetAngle([]() -> int {
		// Get global variables
		int taskState = _taskState;
		double taskDelay = _taskDelay;

		// Delay setting state
		task::sleep(taskDelay * 1000);

		// Set state here
		arm_positionError_radians_to_radiansPerSec_pid.setErrorI(0);
		armStateTargetAngle_degrees = taskState;

		return 1;
	});
}

double getTargetAngle() {
	return armStateTargetAngle_degrees;
}

void setArmStage(int stageId, double delay_sec, double maxSpeed_pct) {
	stageId = aespa_lib::genutil::clamp(stageId, -2, (int) armStages_degrees.size() - 1);
	currentArmStage = stageId;

	// -1 case
	if (stageId == -1) return;

	// Stage max velocity
	armMaxVelocity_pct = maxSpeed_pct;
	if (armMaxVelocity_pct < 1e-5) armMaxVelocity_pct = defaultArmMaxVelocity_pct;

	// Extreme cases
	int stage_value = extremeStages_values[stageId];
	if (stage_value == -2) {
		// Down, hold
		armDownPatience.reset();
		releaseOnExhausted = false;
		setTargetAngle(-1e7, delay_sec);
		return;
	} else if (stage_value == -1) {
		// Down, release
		armDownPatience.reset();
		releaseOnExhausted = true;
		setTargetAngle(-1e7, delay_sec);
		return;
	} else if (stage_value == 1) {
		// Up, release
		armUpPatience.reset();
		releaseOnExhausted = true;
		setTargetAngle(1e7, delay_sec);
		return;
	} else if (stage_value == 2) {
		// Up, hold
		armUpPatience.reset();
		releaseOnExhausted = false;
		setTargetAngle(1e7, delay_sec);
		return;
	}

	// PID stage case
	printf("Arm deg: %.3f\n", armStages_degrees[stageId]);
	setTargetAngle(armStages_degrees[stageId], delay_sec);
}

int getArmStage() {
	return currentArmStage;
}

void resetArmEncoder() {
	armResetted = false;

	// Sanitize rotation sensor's initial value to between [-100, 260]
	setArmPosition(aespa_lib::genutil::modRange(ArmRotationSensor.position(degrees), 360, -100));

	/*
	// Spin downward until exhausted
	setArmStage(0);
	waitUntil(armDownPatience.isExhausted());

	// Sanitize the arm position
	// setArmPosition(0);
	setArmPosition(aespa_lib::genutil::modRange(ArmRotationSensor.position(degrees), 360, -100));
	// */

	// Initialize to default stage
	setArmStage(resetDefaultStageId);
	printf("Default arm: %d\n", resetDefaultStageId);
	armDownPatience.computePatience(ArmRotationSensor.position(degrees) * armEncoder_to_arm_ratio);
	armDownPatience.exhaustNow();

	armResetted = true;
}

bool isArmResetted() {
	return armResetted;
}

void setResetDefaultStage(int stageId) {
	resetDefaultStageId = stageId;
	if (isArmResetted()) {
		setArmStage(stageId);
	}
}

void control(int state) {
	if (canControl()) {
		armStateDirection = state;
		useDirection = true;
	}
}

bool canControl() {
	return controlState;
}

double _taskState;
double _taskDelay;
}


namespace {
void resolveArmExtreme() {
	// Calculate error
	double currentPosition_degrees = ArmRotationSensor.position(degrees) * armEncoder_to_arm_ratio;
	double error_degrees = armStateTargetAngle_degrees - currentPosition_degrees;

	// Spin up or down
	if (error_degrees > 0) {
		/* Elevate */

		// Check patience
		armUpPatience.computePatience(currentPosition_degrees);
		if (armUpPatience.isExhausted()) {
			if (releaseOnExhausted) {
				ArmMotors.stop(coast);
			} else {
				spinArmMotor(3);
			}
			return;
		}

		// Spin
		spinArmMotor(armMaxVelocity_pct);

	} else if (error_degrees < 0) {
		/* Descend */

		// Check patience
		armDownPatience.computePatience(currentPosition_degrees);
		if (armDownPatience.isExhausted()) {
			if (releaseOnExhausted) {
				ArmMotors.stop(coast);
			} else {
				spinArmMotor(-10);
			}
			return;
		}

		// Spin
		spinArmMotor(-armMaxVelocity_pct);
	}
}

void resolveArmDegrees() {
	if (currentArmStage != -2) {
		// -1 case
		if (currentArmStage == -1) {
			return;
		}
	
		// Extreme case
		if (isExtreme()) {
			resolveArmExtreme();
			return;
		}
	}

	/* Position feedback */

	// Calculate error
	aespa_lib::units::PolarAngle currentAngle = ArmRotationSensor.position(degrees) * armEncoder_to_arm_ratio;
	aespa_lib::units::PolarAngle errorAngle = armStateTargetAngle_degrees - currentAngle.polarDeg();
	// printf("Cur: %.3f, target: %.3f\n", currentAngle.polarDeg(), armStateTargetAngle_degrees);

	// Get pid value
	arm_positionError_radians_to_radiansPerSec_pid.computeFromError(errorAngle.polarRad());
	double positionPidVelocity_radiansPerSec = arm_positionError_radians_to_radiansPerSec_pid.getValue();
	// printf("Err: %.3f, pid: %.3f\n", errorAngle.polarDeg(), positionPidVelocity_radiansPerSec);


	/* Feedforward */

	double desiredVelocity_radiansPerSec = positionPidVelocity_radiansPerSec;
	double motorVelocityVolt = arm_velocity_radiansPerSec_to_volt_feedforward.calculateFromMotion(
		aespa_lib::genutil::toRadians(armStateTargetAngle_degrees), desiredVelocity_radiansPerSec, 0
	);
	// double motorVelocityVolt = arm_velocity_radiansPerSec_to_volt_feedforward.calculateFromMotion(currentAngle.polarRad(), desiredVelocity_radiansPerSec, 0);

	/* Position feedback PID */
	arm_positionError_radians_to_volt_pid_feedback.computeFromError(errorAngle.polarRad());
	double feedbackVelocity_volt = arm_positionError_radians_to_volt_pid_feedback.getValue();
	motorVelocityVolt += feedbackVelocity_volt;

	// printf("ARM: %.3f, Err: %.3f\n", currentAngle.polarDeg(), errorAngle.polarDeg());
	double armVelocity_radiansPerSec = ArmMotors.velocity(rpm) * (1.0 / 60.0) * (2 * M_PI);
	// printf("VRS: %.3f %.3f, volt: %.3f\n", desiredVelocity_radiansPerSec, armVelocity_radiansPerSec, motorVelocityVolt);

	/* Command */

	// Clamp velocity
	double motorVelocityPct = aespa_lib::genutil::voltToPct(motorVelocityVolt);
	motorVelocityPct = aespa_lib::genutil::clamp(motorVelocityPct, -armMaxVelocity_pct, armMaxVelocity_pct);
	// printf("MotVal: %.3f\n", motorVelocityPct);

	// Set velocity
	spinArmMotor(motorVelocityPct);
}

void resolveArmDirection() {
	armStateDirection = (armStateDirection > 0) - (armStateDirection < 0);

	switch (armStateDirection) {
		case 1:
			if (ArmMotors.position(deg) > 1000.0) {
				ArmMotors.stop(hold);
			} else {
				ArmMotors.spin(forward, aespa_lib::genutil::pctToVolt(armUpVelocityPct), volt);
			}
			break;

		case -1:
			if (ArmMotors.position(deg) < 10.0) {
				ArmMotors.stop();
			} else {
				ArmMotors.spin(forward, -aespa_lib::genutil::pctToVolt(armMaxVelocity_pct), volt);
			}
			break;

		default:
			ArmMotors.stop(hold);
			break;
	}
}

bool isExtreme() {
	if (!(0 <= currentArmStage && currentArmStage < (int) armStages_degrees.size())) {
		return false;
	}
	return extremeStages_values[currentArmStage] != 0;
}

void setArmPosition(double armEncoderPosition_degrees) {
	// printf("Set Arm Pos: %.3f %.3f %.3f\n", armEncoderPosition_degrees, armEncoderPosition_degrees * armEncoder_to_arm_ratio, ArmRotationSensor.position(degrees));
	ArmRotationSensor.setPosition(armEncoderPosition_degrees, degrees);
	arm_positionError_radians_to_radiansPerSec_pid.resetErrorToZero();
}

void spinArmMotor(double velocityPct) {
	// Spin
	velocityPct = aespa_lib::genutil::clamp(velocityPct, -armMaxVelocity_pct, armMaxVelocity_pct);
	double velocityVolt = aespa_lib::genutil::pctToVolt(velocityPct);
	velocityVolt = aespa_lib::genutil::clamp(velocityVolt, -12, 12);
	ArmMotors.spin(forward, velocityVolt, volt);
	// printf("volt given %.3f, volt: %.3f\n", velocityVolt, ArmMotors.voltage());
	// printf("VVolt: %.3f, temp: %.3f C, torque: %.3f Nm\n", velocityVolt, ArmMotors.temperature(celsius), ArmMotors.torque());
	// printf("pos: %.3f\n", ArmRotationSensor.position(deg) * armEncoder_to_arm_ratio);
	// ArmMotors.spin(forward, velocityPct, pct);
}
}
