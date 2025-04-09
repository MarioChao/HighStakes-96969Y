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
double armMaxVelocity_rpm = 66;
double armMaxVelocity_radiansPerSec = armMaxVelocity_rpm * (1.0 / 60.0) * (2 * M_PI);

double armEncoder_to_arm_ratio = 1.0 / 3.0;

// Stage controllers
ArmFeedforward arm_velocity_radiansPerSec_to_volt_feedforward(0.1, 0.3, 12.0 / armMaxVelocity_radiansPerSec);
PIDController arm_positionError_radians_to_radiansPerSec_Pid(10.0, 0, 0.25);
PIDController arm_positionError_degrees_to_volt_pid(0.35, 0, 0);
PatienceController armUpPatience(12, 1.0, true, 5);
PatienceController armDownPatience(6, 1.0, false, 5);

// Stage config
std::vector<double> armStages_degrees = { 0, 4, 15, 40, 0 };
std::vector<int> extremeStages_values = { -1, 0, 0, 0, 1 };
int currentArmStage = 0;
bool releaseOnExhausted = true;

// Reset arm info
bool armResetted = false;
int resetDefaultStageId = 0;

// Speed config
double armVelocityPct = 100;
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
		arm_positionError_radians_to_radiansPerSec_Pid.setErrorI(0);
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
		arm_positionError_radians_to_radiansPerSec_Pid.setErrorI(0);
		armStateTargetAngle_degrees = taskState;

		return 1;
	});
}

void setArmStage(int stageId, double delaySec) {
	stageId = aespa_lib::genutil::clamp(stageId, 0, (int) armStages_degrees.size() - 1);
	currentArmStage = stageId;

	// Extreme cases
	int stage_value = extremeStages_values[stageId];
	if (stage_value == -2) {
		// Down, hold
		armDownPatience.reset();
		releaseOnExhausted = false;
		setTargetAngle(-1e7, delaySec);
		return;
	} else if (stage_value == -1) {
		// Down, release
		armDownPatience.reset();
		releaseOnExhausted = true;
		setTargetAngle(-1e7, delaySec);
		return;
	} else if (stage_value == 1) {
		// Up, release
		armUpPatience.reset();
		releaseOnExhausted = true;
		setTargetAngle(1e7, delaySec);
		return;
	} else if (stage_value == 2) {
		// Up, hold
		armUpPatience.reset();
		releaseOnExhausted = false;
		setTargetAngle(1e7, delaySec);
		return;
	}

	// PID stage case
	setTargetAngle(armStages_degrees[stageId], delaySec);
}

int getArmStage() {
	return currentArmStage;
}

void resetArmEncoder() {
	armResetted = false;

	// Sanitize rotation sensor's initial value to between [-100, 260]
	setArmPosition(aespa_lib::genutil::modRange(ArmRotationSensor.position(degrees), 360, -100));

	// /*
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
		spinArmMotor(armVelocityPct);

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
		spinArmMotor(-armVelocityPct);
	}
}

void resolveArmDegrees() {
	// Extreme case
	if (isExtreme()) {
		resolveArmExtreme();
		return;
	}


	/* Position feedback */

	// Calculate error
	aespa_lib::units::PolarAngle currentAngle = ArmRotationSensor.position(degrees) * armEncoder_to_arm_ratio;
	aespa_lib::units::PolarAngle errorAngle = armStateTargetAngle_degrees - currentAngle.polarDeg();
	// printf("Cur: %.3f, target: %.3f\n", currentAngle.polarDeg(), armStateTargetAngle_degrees);

	// Get pid value
	arm_positionError_radians_to_radiansPerSec_Pid.computeFromError(errorAngle.polarRad());
	double positionPidVelocity_radiansPerSec = arm_positionError_radians_to_radiansPerSec_Pid.getValue();
	// printf("Err: %.3f, pid: %.3f\n", errorAngle.polarDeg(), positionPidVelocity_radiansPerSec);


	/* Feedforward */

	double desiredVelocity_radiansPerSec = positionPidVelocity_radiansPerSec;
	double motorVelocityVolt = arm_velocity_radiansPerSec_to_volt_feedforward.calculateFromMotion(currentAngle.polarRad(), desiredVelocity_radiansPerSec, 0);


	/* PID overwrite */

	if (false) {
		arm_positionError_degrees_to_volt_pid.computeFromError(errorAngle.polarDeg());
		motorVelocityVolt = arm_positionError_degrees_to_volt_pid.getValue();
	}

	// printf("ARM: %.3f, Err: %.3f\n", currentAngle.polarDeg(), errorAngle.polarDeg());
	double armVelocity_radiansPerSec = ArmMotors.velocity(rpm) * (1.0 / 60.0) * (2 * M_PI);
	// printf("VRS: %.3f %.3f, volt: %.3f\n", desiredVelocity_radiansPerSec, armVelocity_radiansPerSec, motorVelocityVolt);

	/* Command */

	// Clamp velocity
	double motorVelocityPct = aespa_lib::genutil::voltToPct(motorVelocityVolt);
	motorVelocityPct = aespa_lib::genutil::clamp(motorVelocityPct, -armVelocityPct, armVelocityPct);
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
				ArmMotors.spin(forward, -aespa_lib::genutil::pctToVolt(armVelocityPct), volt);
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
	arm_positionError_radians_to_radiansPerSec_Pid.resetErrorToZero();
}

void spinArmMotor(double velocityPct) {
	// Spin
	double velocityVolt = aespa_lib::genutil::pctToVolt(velocityPct);
	velocityVolt = aespa_lib::genutil::clamp(velocityVolt, -12, 12);
	ArmMotors.spin(forward, velocityVolt, volt);
	// printf("VVolt: %.3f, temp: %.3f C, torque: %.3f Nm\n", velocityVolt, ArmMotors.temperature(celsius), ArmMotors.torque());
	// printf("pos: %.3f\n", ArmRotationSensor.position(deg) * armEncoder_to_arm_ratio);
	// ArmMotors.spin(forward, velocityPct, pct);
}
}
