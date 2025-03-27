#include "global-vars.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"


namespace {
using aespa_lib::sensor_beats::RotationSensor;
using aespa_lib::sensor_beats::OpticalShaftEncoder;
using aespa_lib::sensor_beats::Motor;
using aespa_lib::sensor_beats::TrackingWheel;
using pas1_lib::chassis_tracker::Odometry;

using pas1_lib::basic_control::chassis::Differential;
using pas1_lib::basic_control::chassis::AutonSettings;
using pas1_lib::auton::control_loops::ForwardController;
using pas1_lib::auton::control_loops::PIDController;
using pas1_lib::auton::end_conditions::PatienceController;

using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::profiles::SplineProfile;
using aespa_lib::datas::NamedStorage;
}


// ---------- Global variables ----------

competition Competition;

int intakePartType = 1;

bool isArmPneumatic = false;

timer drivingTimer;


/* Odometry */

namespace {

// Sensors
RotationSensor look_rotationBeats(LookRotation);
RotationSensor right_rotationBeats(RightRotation);
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
TrackingWheel right1_trackingWheel(right_rotationBeats, 180, 1, 2.0, 0);
TrackingWheel right2_trackingWheel(right_opticalBeats, 180, 1, 2.75, -3.5);

}

// Odometry
Odometry mainOdometry = Odometry()
.addTrackingWheel(lookLeft_trackingWheel)
.addTrackingWheel(lookRight_trackingWheel)
// .addTrackingWheel(look1_trackingWheel)
.addTrackingWheel(right1_trackingWheel)
.addInertialSensor(InertialSensor, 0, 0)
.setPositionFactor(1.0 / field::tileLengthIn)
;


/* Chassis */

namespace {
AutonSettings autonSettings(
	ForwardController(1.0, 3.1875, 0.4), // feedforward
	PIDController(400, 0, 0, 0.04), // position feedback
	PIDController(1.8, 0, 0.005), // velocity feedback
	PIDController(70, 0, 0, 0.04), // linear pid
	PIDController(1.0, 0, 0, 5), // angular pid
	PatienceController(4, 0.5, false), // linear patience
	PatienceController(40, 0.5, false), // angular patience
	false // reverse
);
}

Differential robotChassis = Differential(
	mainOdometry, autonSettings,
	LeftMotors, RightMotors
);


/* Autonomous */

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;

// Spline Profile Storage
NamedStorage<SplineProfile> splineProfile_storage;
