#include "global-vars.h"
#include "Utilities/fieldInfo.h"


namespace {
using aespa_lib::sensor_beats::RotationSensor;
using aespa_lib::sensor_beats::OpticalShaftEncoder;
using aespa_lib::sensor_beats::Motor;
using aespa_lib::sensor_beats::TrackingWheel;

using pas1_lib::chassis::settings::Odometry;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using pas1_lib::chassis::base::Differential;
using pas1_lib::auton::control_loops::ForwardController;
using pas1_lib::auton::control_loops::PIDController;
using pas1_lib::auton::control_loops::SlewController;
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


/* Bot Info */

BotInfo botInfo(
	24, // track width (holes)
	3.25, // wheel diameter (inches)
	36.0 / 36.0, // wheel to motor gear ratio
	600, // motor rpm
	600 // motor rpm/s
);


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
	lookLeft_motorBeats, 90, botInfo.motorToWheel_gearRatio, 2.75, -botInfo.trackWidth_inches / 2
);
TrackingWheel lookRight_trackingWheel(
	lookRight_motorBeats, 90, botInfo.motorToWheel_gearRatio, 2.75, botInfo.trackWidth_inches / 2
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
	ForwardController(1.0, 12.0 / botInfo.maxVel_tilesPerSec, 0.4), // feedforward (tiles/sec to volt)
	PIDController(15.06, 0, 0, 0.04), // position feedback (tiles to tiles/sec)
	PIDController(5.184, 0, 0), // velocity feedback (tiles/sec to volt)
	PIDController(70, 0, 0, 0.04), // linear pid (tiles to pct)
	PIDController(3.5, 0, 0.2, 3), // angular pid (degrees to pct)
	SlewController(200), // linear slew (pct/sec)
	SlewController(200), // angular slew (pct/sec)
	PatienceController(10, 0.001, false), // linear patience (tiles)
	PatienceController(100, 0.5, false), // angular patience (degrees)
	false // relative rotation
);
}

Differential robotChassis = Differential(
	mainOdometry, botInfo, autonSettings,
	LeftMotors, RightMotors
);


/* Autonomous */

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;

// Spline Profile Storage
NamedStorage<SplineProfile> splineProfile_storage;
