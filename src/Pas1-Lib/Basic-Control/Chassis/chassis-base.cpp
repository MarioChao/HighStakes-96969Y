#include "Pas1-Lib/Basic-Control/Chassis/chassis-base.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"

namespace {
using aespa_lib::datas::Linegular;
}


namespace pas1_lib {
namespace basic_control {
namespace chassis {

void ChassisBase::control_local2d(
	double right_pct, double look_pct,
	double angular_pct
) {}

void ChassisBase::control_global2d(
	double right_pct, double look_pct,
	double angular_pct
) {
	Linegular globalMove(right_pct, look_pct, 0);
	globalMove.rotateXYBy(-getLookPose().getThetaPolarAngle_radians());
	control_local2d(globalMove.getX(), globalMove.getY(), angular_pct);
}

void ChassisBase::control() {}

void ChassisBase::stopMotors(brakeType mode) {}

void ChassisBase::setLookPose(Linegular pose) {
	odometry.setLookPose_scaled(pose);
}

Linegular ChassisBase::getLookPose() {
	return odometry.getLookPose_scaled();
}

}
}
}
