#include "Autonomous/auton.h"

#include "Autonomous/autonpaths.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Utilities/debugFunctions.h"
#include "preauton.h"
#include "main.h"

namespace {
	using namespace auton;

	bool userRunningAutonomous = false;
	autonomousType auton_runType = autonomousType::LoveShape;
	int auton_allianceId;
}

namespace auton {
	void setAutonRunType(int allianceId, autonomousType autonType) {
		switch (autonType) {
			case autonomousType::RedUp:
				debug::printOnController("Auton: RedUp");
				printf("RedUp\n");
				botintake::setFilterColor("blue");
				botintake2::setFilterColor("blue");
				break;
			case autonomousType::RedDown:
				debug::printOnController("Auton: RedDown");
				printf("RedDown\n");
				botintake::setFilterColor("blue");
				botintake2::setFilterColor("blue");
				break;
			case autonomousType::BlueUp:
				debug::printOnController("Auton: BlueUp");
				printf("BlueUp\n");
				botintake::setFilterColor("red");
				botintake2::setFilterColor("red");
				break;
			case autonomousType::BlueDown:
				debug::printOnController("Auton: BlueDown");
				printf("BlueDown\n");
				botintake::setFilterColor("red");
				botintake2::setFilterColor("red");
				break;
			case autonomousType::RedUpSafe:
				debug::printOnController("Auton: RedUp SF");
				printf("RedUp Safe\n");
				botintake::setFilterColor("blue");
				botintake2::setFilterColor("blue");
				break;
			case autonomousType::RedDownSafe:
				debug::printOnController("Auton: RedDown SF");
				printf("RedDown Safe\n");
				botintake::setFilterColor("blue");
				botintake2::setFilterColor("blue");
				break;
			case autonomousType::BlueUpSafe:
				debug::printOnController("Auton: BlueUp SF");
				printf("BlueUp Safe\n");
				botintake::setFilterColor("red");
				botintake2::setFilterColor("red");
				break;
			case autonomousType::BlueDownSafe:
				debug::printOnController("Auton: BlueDown SF");
				printf("BlueDown Safe\n");
				botintake::setFilterColor("red");
				botintake2::setFilterColor("red");
				break;
			case autonomousType::AutonSkills:
				debug::printOnController("Auton: Skills");
				printf("AuSk\n");
				botintake::setFilterColor("blue");
				botintake2::setFilterColor("blue");
				break;
			case autonomousType::DrivingSkills:
				debug::printOnController("Driving Skills");
				printf("DrSk\n");
				botintake::setFilterColor("blue");
				botintake2::setFilterColor("blue");
				break;
			default:
				debug::printOnController("Auton: None");
				printf("None\n");
				botintake::setFilterColor("none");
				botintake2::setFilterColor("none");
				break;
		}
		auton_runType = autonType;
		auton_allianceId = allianceId;
	}


	void showAutonRunType() {
		setAutonRunType(auton_allianceId, auton_runType);
	}


	autonomousType getAutonRunType() {
		return auton_runType;
	}


	bool isUserRunningAuton() {
		return userRunningAutonomous;
	}


	void runAutonomous() {
		printf("Auton time!\n");
		userRunningAutonomous = false;
		switch (auton_runType) {
			case autonomousType::RedUp:
				autonpaths::runAutonRedUp();
				break;
			case autonomousType::RedDown:
				autonpaths::runAutonRedDown();
				break;
			case autonomousType::BlueUp:
				autonpaths::runAutonBlueUp();
				break;
			case autonomousType::BlueDown:
				autonpaths::runAutonBlueDown();
				break;
			case autonomousType::RedUpSafe:
				autonpaths::runAutonRedUpSafe();
				break;
			case autonomousType::RedDownSafe:
				autonpaths::runAutonRedDownOld();
				break;
			case autonomousType::BlueUpSafe:
				autonpaths::runAutonBlueUpSafe();
				break;
			case autonomousType::BlueDownSafe:
				autonpaths::runAutonBlueDownOld();
				break;
			case autonomousType::AutonSkills:
				autonpaths::runAutonSkills();
				break;
			case autonomousType::AllianceWallStake:
				autonpaths::runAllianceWallStake();
				break;
			case autonomousType::LoveShape:
				autonpaths::runLoveShape();
				break;
			case autonomousType::FieldTour:
				autonpaths::runFieldTour();
				break;
			case autonomousType::Test:
				autonpaths::autonTest();
				break;
			case autonomousType::OdometryRadiusTest:
				autonpaths::odometryRadiusTest();
				break;
			default:
				break;
		}
	}
}
