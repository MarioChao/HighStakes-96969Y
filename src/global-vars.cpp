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
	sensor_beats::OpticalShaftEncoder rightEncoder_beats(RightEncoder);
	sensor_beats::Motor lookLeftMotor_beats(LeftMotorA);
	sensor_beats::Motor lookRightMotor_beats(RightMotorA);

	// Tracking wheels
	chassis_tracker::TrackingWheel lookL_trackingWheel(
		lookLeftMotor_beats, 90, botinfo::driveMotorToWheel_gearRatio, 2.75, -botinfo::halfRobotLengthIn
	);
	chassis_tracker::TrackingWheel lookR_trackingWheel(
		lookRightMotor_beats, 90, botinfo::driveMotorToWheel_gearRatio, 2.75, botinfo::halfRobotLengthIn
	);
	// chassis_tracker::TrackingWheel look_trackingWheel(lookRotation_beats, -90, 1, 2.0, 0);
	// chassis_tracker::TrackingWheel right_trackingWheel(rightEncoder_beats, 180, 1, 2.75, -3.5);
}

// Odometry
chassis_tracker::Odometry mainOdometry = chassis_tracker::Odometry(
	{lookL_trackingWheel, lookR_trackingWheel}
)
.addInertialSensor(InertialSensor, 0, 0)
.setPositionFactor(1.0 / field::tileLengthIn)
;

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;
