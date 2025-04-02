#pragma once

#include <cmath>
#include "Aespa-Lib/Winter-Utilities/units.h"


namespace {
const double inchesPerTile = 23 + 13.0 / 16;
}


namespace pas1_lib {
namespace chassis {
namespace settings {


struct BotInfo {
	BotInfo(
		double trackWidth_holes,
		double wheelDiameter_inches,
		double wheelToMotor_gearRatio,
		double maxMotorSpeed_rpm,
		double maxMotorAcceleration_rpmPerSec
	)
		: trackWidth_holes(trackWidth_holes),
		wheelDiameter_inches(wheelDiameter_inches),
		wheelToMotor_gearRatio(wheelToMotor_gearRatio),
		maxMotorSpeed_rpm(maxMotorSpeed_rpm),
		maxMotorAcceleration_rpmPerSec(maxMotorAcceleration_rpmPerSec),

		trackWidth_inches(trackWidth_holes / 2.0),
		trackWidth_tiles(trackWidth_inches / inchesPerTile),
		wheelDiameter_tiles(wheelDiameter_inches / inchesPerTile),
		wheelCircum_tiles(M_PI * wheelDiameter_tiles),
		motorToWheel_gearRatio(1.0 / wheelToMotor_gearRatio),
		maxVel_tilesPerSec(
			maxMotorSpeed_rpm * (1.0 / 60.0)
			* motorToWheel_gearRatio * (wheelCircum_tiles / 1.0)
		),
		tilesPerSecond_to_pct((1.0 / maxVel_tilesPerSec) * 100),
		maxAccel_tilesPerSec2(
			maxMotorAcceleration_rpmPerSec * (1.0 / 60.0)
			* motorToWheel_gearRatio * (wheelCircum_tiles / 1.0)
		)
		{}

	// Specified info
	const double trackWidth_holes;
	const double wheelDiameter_inches;
	const double wheelToMotor_gearRatio;
	const double maxMotorSpeed_rpm;
	const double maxMotorAcceleration_rpmPerSec;
	
	// Derived info
	const double trackWidth_inches;
	const double trackWidth_tiles;
	
	const double wheelDiameter_tiles;
	const double wheelCircum_tiles;

	const double motorToWheel_gearRatio;

	const double maxVel_tilesPerSec;
	const double tilesPerSecond_to_pct;

	const double maxAccel_tilesPerSec2;
};


}
}
}
