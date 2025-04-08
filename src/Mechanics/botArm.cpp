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

void setArmPosition(double position_degrees);
void spinArmMotor(double velocityPct);

// Motor config
double armMotorMaxVelocity_rpm = 100;
double armMotorMaxVelocity_radiansPerSec = armMotorMaxVelocity_rpm * (1.0 / 60.0) * (2 * M_PI);

// Stage controllers
ArmFeedforward arm_velocity_radiansPerSec_to_volt_feedforward(1.0, 1.7, 12.0 / armMotorMaxVelocity_radiansPerSec);
PIDController arm_positionError_radians_to_radiansPerSec_Pid(18.0, 0, 0);
// PIDController arm_positionError_degrees_to_volt_pid(1.9, 0, 0);
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
			// printf("st: %d, armvolt: %.3f\n", currentArmStage, ArmMotor.voltage(volt));
			// printf("arm torque: %.3f Nm\n", ArmMotor.torque());
			resolveArmDegrees();
		}

		wait(10, msec);
	}
}

void preauton() {
	ArmMotor.setPosition(0, degrees);
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
	setArmPosition(aespa_lib::genutil::modRange(ArmRotationSensor.angle(degrees), 360, -100));

	/*
	// Spin downward until exhausted
	setArmStage(0);
	waitUntil(armDownPatience.isExhausted());

	// Set the position as 0 degrees
	setArmPosition(0);
	// */

	// Initialize to default stage
	setArmStage(resetDefaultStageId);
	printf("Default arm: %d\n", resetDefaultStageId);
	armDownPatience.computePatience(ArmRotationSensor.position(degrees));
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
	double currentPosition_degrees = ArmRotationSensor.position(degrees);
	double error_degrees = armStateTargetAngle_degrees - currentPosition_degrees;

	// Spin up or down
	if (error_degrees > 0) {
		/* Elevate */

		// Check patience
		armUpPatience.computePatience(currentPosition_degrees);
		if (armUpPatience.isExhausted()) {
			if (releaseOnExhausted) {
				ArmMotor.stop(coast);
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
				ArmMotor.stop(coast);
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
	aespa_lib::units::PolarAngle currentAngle = ArmRotationSensor.position(degrees);
	aespa_lib::units::PolarAngle errorAngle = armStateTargetAngle_degrees - currentAngle.polarDeg();
	// printf("Cur: %.3f, target: %.3f\n", currentAngle.polarDeg(), armStateTargetAngle_degrees);

	// Get pid value
	arm_positionError_radians_to_radiansPerSec_Pid.computeFromError(errorAngle.polarRad());
	double positionPidVelocity_radiansPerSec = arm_positionError_radians_to_radiansPerSec_Pid.getValue();
	// printf("Err: %.3f, pid: %.3f\n", errorAngle.polarDeg(), positionPidVelocity_radiansPerSec);


	/* Feedforward */

	double desiredVelocity_radiansPerSec = positionPidVelocity_radiansPerSec;
	double motorVelocityVolt = arm_velocity_radiansPerSec_to_volt_feedforward.calculateFromMotion(currentAngle.polarRad(), desiredVelocity_radiansPerSec, 0);


	// arm_positionError_degrees_to_volt_pid.computeFromError(errorAngle.polarDeg());
	// motorVelocityVolt = arm_positionError_degrees_to_volt_pid.getValue();

	// printf("ARM: %.3f, Err: %.3f\n", currentAngle.polarDeg(), errorAngle.polarDeg());
	double armVelocity_radiansPerSec = ArmMotor.velocity(rpm) * (1.0 / 60.0) * (2 * M_PI);
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
			if (ArmMotor.position(deg) > 1000.0) {
				ArmMotor.stop(hold);
			} else {
				ArmMotor.spin(forward, aespa_lib::genutil::pctToVolt(armUpVelocityPct), volt);
			}
			break;

		case -1:
			if (ArmMotor.position(deg) < 10.0) {
				ArmMotor.stop();
			} else {
				ArmMotor.spin(forward, -aespa_lib::genutil::pctToVolt(armVelocityPct), volt);
			}
			break;

		default:
			ArmMotor.stop(hold);
			break;
	}
}

bool isExtreme() {
	if (!(0 <= currentArmStage && currentArmStage < (int) armStages_degrees.size())) {
		return false;
	}
	return extremeStages_values[currentArmStage] != 0;
}

void setArmPosition(double position_degrees) {
	ArmRotationSensor.setPosition(position_degrees, degrees);
	arm_positionError_radians_to_radiansPerSec_Pid.resetErrorToZero();
}

void spinArmMotor(double velocityPct) {
	// Spin
	double velocityVolt = aespa_lib::genutil::pctToVolt(velocityPct);
	velocityVolt = aespa_lib::genutil::clamp(velocityVolt, -12, 12);
	ArmMotor.spin(forward, velocityVolt, volt);
	// printf("VVolt: %.3f, temp: %.3f C, torque: %.3f Nm\n", velocityVolt, ArmMotor.temperature(celsius), ArmMotor.torque());
	// ArmMotor.spin(forward, velocityPct, pct);
}
}
