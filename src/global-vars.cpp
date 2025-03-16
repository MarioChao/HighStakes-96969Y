#include "global-vars.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"

using aespa_lib::sensor_beats::RotationSensor;
using aespa_lib::sensor_beats::OpticalShaftEncoder;
using aespa_lib::sensor_beats::Motor;
using aespa_lib::sensor_beats::TrackingWheel;
using pas1_lib::chassis_tracker::Odometry;


// ---------- Global variables ----------

competition Competition;

int intakePartType = 1;

bool isArmPneumatic = false;

timer drivingTimer;


/* Odometry */

namespace {

// Sensors
RotationSensor look_rotationBeats(LookRotation);
OpticalShaftEncoder right_opticalBeats(RightEncoder);
Motor lookLeft_motorBeats(LeftMotorA);
Motor lookRight_motorBeats(RightMotorA);

// Tracking wheels
TrackingWheel lookLeft_trackingWheel(
	lookLeft_motorBeats, 90, botinfo::driveMotorToWheel_gearRatio, 2.75, -botinfo::halfRobotLengthIn
);
TrackingWheel lookRight_trackingWheel(
	lookRight_motorBeats, 90, botinfo::driveMotorToWheel_gearRatio, 2.75, botinfo::halfRobotLengthIn
);
TrackingWheel look1_trackingWheel(look_rotationBeats, -90, 1, 2.0, 0);
TrackingWheel right1_trackingWheel(right_opticalBeats, 180, 1, 2.75, -3.5);

}

// Odometry
Odometry mainOdometry = Odometry()
.addTrackingWheel(lookLeft_trackingWheel)
.addTrackingWheel(lookRight_trackingWheel)
// .addTrackingWheel(look1_trackingWheel)
// .addTrackingWheel(right1_trackingWheel)
.addInertialSensor(InertialSensor, 0, 0)
.setPositionFactor(1.0 / field::tileLengthIn)
;


/* Autonomous */

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner_Old testTrajectoryPlan;
timer trajectoryTestTimer;
