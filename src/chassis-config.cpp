#include "chassis-config.h"

#include "Utilities/fieldInfo.h"


namespace {
using aespa_lib::sensor_beats::RotationSensor;
using aespa_lib::sensor_beats::OpticalShaftEncoder;
using aespa_lib::sensor_beats::Motor;
using aespa_lib::sensor_beats::TrackingWheel;
using namespace aespa_lib::units::angle;

using pas1_lib::chassis::settings::Odometry;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using pas1_lib::chassis::base::Differential;
using pas1_lib::auton::control_loops::SimpleFeedforward;
using pas1_lib::auton::control_loops::PIDController;
using pas1_lib::auton::control_loops::SlewController;
using pas1_lib::auton::end_conditions::PatienceController;
}


// ---------- Chassis configuration ----------

/* Bot Info */

BotInfo botInfo(
	23, // track width (holes)
	3.25, // wheel diameter (inches)
	48.0 / 36.0, // wheel to motor gear ratio
	600, // motor rpm
	600 * 1.5 // motor rpm/s
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
	lookLeft_motorBeats, 90_polarDeg, botInfo.motorToWheel_gearRatio, botInfo.wheelDiameter_inches, -botInfo.trackWidth_inches / 2
);
TrackingWheel lookRight_trackingWheel(
	lookRight_motorBeats, 90_polarDeg, botInfo.motorToWheel_gearRatio, botInfo.wheelDiameter_inches, botInfo.trackWidth_inches / 2
);
TrackingWheel look1_trackingWheel(look_rotationBeats, 90_polarDeg, 1, 2.75, -0.013);
TrackingWheel right1_trackingWheel(right_rotationBeats, 0_polarDeg, 1, 2.75, 2.873);

}

// Odometry
Odometry mainOdometry = Odometry()
.addTrackingWheel(lookLeft_trackingWheel)
.addTrackingWheel(lookRight_trackingWheel)
// .addTrackingWheel(look1_trackingWheel)
.addTrackingWheel(right1_trackingWheel)
.addInertialSensor(InertialSensor, 0, 0)
.setPositionFactor(1.0 / field::tileLengthIn);


/* Chassis */

namespace {

// Auton settings
AutonSettings autonSettings(
	SimpleFeedforward(0, 12.0 / botInfo.maxVel_tilesPerSec, 0.03), // feedforward (tiles/sec to volt)
	PIDController(3.0), // position feedback (tiles to tiles/sec)
	PIDController(1.0), // velocity feedback (tiles/sec to volt)
	// PIDController(0.0), // velocity feedback (tiles/sec to volt)
	PIDController(100, 0, 0, 0.06, 0.05), // linear pid (tiles to pct)
	// PIDController(70, 0, 0, 0.03, 0.05), // linear pid (tiles to pct)
	PIDController(3.25, 0, 0.225, 2, 0.05), // angular pid (degrees to pct)
	SlewController(100), // linear slew (pct/sec)
	SlewController(100), // angular slew (pct/sec)
	SlewController(120), // motor slew (pct/sec)
	PatienceController(30, 0.001, false), // linear patience (tiles)
	PatienceController(30, 0.5, false), // angular patience (degrees)
	false // relative rotation
);
}

// Chassis
Differential robotChassis = Differential(
	mainOdometry, botInfo, autonSettings,
	LeftMotors, RightMotors
);
