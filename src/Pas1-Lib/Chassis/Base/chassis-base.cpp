#include "Pas1-Lib/Chassis/Base/chassis-base.h"


namespace {
using aespa_lib::geometry::Vector2D;
using aespa_lib::datas::Linegular;
using namespace aespa_lib::units::literals::angle;
}


namespace pas1_lib {
namespace chassis {
namespace base {

ChassisBase::ChassisBase(settings::Odometry &odometry, settings::BotInfo &botInfo, settings::AutonSettings &autonSettings)
	: botInfo(botInfo), autonSettings(autonSettings), odometry(odometry) {
	overwriteLookVelocity = { false, 0 };
}

void ChassisBase::control_local2d(
	double right_pct, double look_pct,
	double angular_pct,
	bool useSlew
) {}

void ChassisBase::control_global2d(
	double right_pct, double look_pct,
	double angular_pct,
	bool useSlew
) {
	Vector2D localMove(right_pct, look_pct);
	localMove.rotateBy(90_polarDeg - getLookPose().getRotation());
	control_local2d(localMove.x, localMove.y, angular_pct, useSlew);
}

void ChassisBase::control() {}

void ChassisBase::stopMotors(brakeType mode) {}

void ChassisBase::setLookPose(Linegular pose) {
	odometry.setLookPose_scaled(pose);
}

Linegular ChassisBase::getLookPose() {
	return odometry.getLookPose_scaled();
}

aespa_lib::units::PolarAngle ChassisBase::getLookRotation() {
	return odometry.getLookRotation();
}

double ChassisBase::getLookVelocity() {
	return 0;
}

double ChassisBase::getAngularVelocity() {
	return 0;
}

}
}
}
