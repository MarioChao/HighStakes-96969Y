#pragma once

#include "Pas1-Lib/Auton/Control-Loops/pid.h"
#include "Pas1-Lib/Auton/Control-Loops/feedforward.h"
#include "Pas1-Lib/Auton/Control-Loops/slew.h"
#include "Pas1-Lib/Auton/End-Conditions/patience.h"


namespace pas1_lib {
namespace chassis {
namespace settings {


struct AutonSettings {
	AutonSettings(
		auton::control_loops::ForwardController ff_velocity_tilesPerSec_to_volt_feedforward,

		auton::control_loops::PIDController fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid,
		auton::control_loops::PIDController fb_velocityError_tilesPerSec_to_volt_pid,

		auton::control_loops::PIDController distanceError_tiles_to_velocity_pct_pid,

		auton::control_loops::PIDController angleError_degrees_to_velocity_pct_pid,

		auton::end_conditions::PatienceController distanceError_tiles_patience,
		auton::end_conditions::PatienceController angleError_degrees_patience,

		bool useRelativeRotation
	)
		: ff_velocity_tilesPerSec_to_volt_feedforward(ff_velocity_tilesPerSec_to_volt_feedforward),
		fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid(fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid),
		fb_velocityError_tilesPerSec_to_volt_pid(fb_velocityError_tilesPerSec_to_volt_pid),
		distanceError_tiles_to_velocity_pct_pid(distanceError_tiles_to_velocity_pct_pid),
		angleError_degrees_to_velocity_pct_pid(angleError_degrees_to_velocity_pct_pid),
		distanceError_tiles_patience(distanceError_tiles_patience),
		angleError_degrees_patience(angleError_degrees_patience),
		useRelativeRotation(useRelativeRotation) {}

	// Feedforward
	auton::control_loops::ForwardController ff_velocity_tilesPerSec_to_volt_feedforward;

	// Linear feedback
	auton::control_loops::PIDController fb_distanceError_tiles_to_deltaVelocity_tilesPerSec_pid;
	auton::control_loops::PIDController fb_velocityError_tilesPerSec_to_volt_pid;

	// Linear PID
	auton::control_loops::PIDController distanceError_tiles_to_velocity_pct_pid;

	// Angular PID
	auton::control_loops::PIDController angleError_degrees_to_velocity_pct_pid;

	// Patience
	auton::end_conditions::PatienceController distanceError_tiles_patience;
	auton::end_conditions::PatienceController angleError_degrees_patience;

	// Booleans
	bool useRelativeRotation;
};


}
}
}
