#include "global-vars.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"

// ---------- Global variables ----------

competition Competition;

int intakePartType = 1;

bool isArmPneumatic = false;

timer drivingTimer;


/* Odometry */

namespace {
	// Sensors
	sensor_beats::RotationSensor lookRotation_beats(LookRotation);
	sensor_beats::OpticalShaftEncoder rightOptical_beats(RightEncoder);
	sensor_beats::Motor lookLeftMotor_beats(LeftMotorA);
	sensor_beats::Motor lookRightMotor_beats(RightMotorA);

	// Tracking wheels
	chassis_tracker::TrackingWheel lookLeft_trackingWheel(
		lookLeftMotor_beats, 90, botinfo::driveMotorToWheel_gearRatio, 2.75, -botinfo::halfRobotLengthIn
	);
	chassis_tracker::TrackingWheel lookRight_trackingWheel(
		lookRightMotor_beats, 90, botinfo::driveMotorToWheel_gearRatio, 2.75, botinfo::halfRobotLengthIn
	);
	chassis_tracker::TrackingWheel lookRotation_trackingWheel(lookRotation_beats, -90, 1, 2.0, 0);
	chassis_tracker::TrackingWheel rightOptical_trackingWheel(rightOptical_beats, 180, 1, 2.75, -3.5);
}

// Odometry
chassis_tracker::Odometry mainOdometry = chassis_tracker::Odometry()
.addTrackingWheel(lookLeft_trackingWheel)
.addTrackingWheel(lookRight_trackingWheel)
.addTrackingWheel(lookRotation_trackingWheel)
.addTrackingWheel(rightOptical_trackingWheel)
.addInertialSensor(InertialSensor, 0, 0)
.setPositionFactor(1.0 / field::tileLengthIn)
;

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;
