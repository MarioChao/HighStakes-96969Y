#pragma once

#include "vex.h"
#include "Pas1-Lib/Chassis/Settings/odometry.h"
#include "Pas1-Lib/Chassis/Settings/bot-info.h"
#include "Pas1-Lib/Chassis/Settings/auton-settings.h"
#include "Pas1-Lib/Chassis/Settings/motion-handler.h"
#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace pas1_lib {
namespace chassis {
namespace base {

class ChassisBase {
public:
	ChassisBase(settings::Odometry &odometry, settings::BotInfo &botInfo, settings::AutonSettings &autonSettings);

	virtual void control_local2d(
		double right_pct, double look_pct,
		double angular_pct,
		bool useSlew = false
	);
	void control_global2d(
		double right_pct, double look_pct,
		double angular_pct,
		bool useSlew = false
	);
	virtual void control();
	virtual void stopMotors(brakeType mode);

	void setLookPose(aespa_lib::datas::Linegular pose);
	aespa_lib::datas::Linegular getLookPose();
	aespa_lib::units::PolarAngle getLookRotation();

	virtual double getLookVelocity();
	virtual double getAngularVelocity();


	settings::BotInfo &botInfo;
	settings::AutonSettings &autonSettings;
	settings::MotionHandler motionHandler;

	std::pair<bool, double> overwriteLookVelocity;

private:
	settings::Odometry &odometry;
};

}
}
}
