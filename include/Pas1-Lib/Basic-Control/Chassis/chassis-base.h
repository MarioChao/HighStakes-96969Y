#pragma once

#include "vex.h"
#include "Pas1-Lib/Chassis-Tracker/odometry.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace pas1_lib {
namespace basic_control {
namespace chassis {

class ChassisBase {
public:
	virtual void control_local2d(
		double right_pct, double look_pct,
		double angular_pct
	);
	void control_global2d(
		double right_pct, double look_pct,
		double angular_pct
	);
	virtual void control();
	virtual void stopMotors(brakeType mode);

	void setLookPose(aespa_lib::datas::Linegular pose);
	aespa_lib::datas::Linegular getLookPose();

private:
	chassis_tracker::Odometry odometry;
};

}
}
}
