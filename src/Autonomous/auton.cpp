#include "Autonomous/auton.h"

#include "Autonomous/autonpaths.h"
#include "Mechanics/botIntake.h"
#include "Utilities/debugFunctions.h"
#include "preauton.h"
#include "main.h"

namespace {
	using namespace auton;

	using debug::printOnController;

	bool userRunningAutonomous = false;
	autonomousType auton_runType = autonomousType::DrivingSkills;
	int auton_allianceId;
}

namespace auton {
	void setAutonRunType(int allianceId, autonomousType autonType) {
		switch (autonType) {
			case autonomousType::RedUp:
				printOnController("Auton: RedUp");
				printf("RedUp\n");
				botintake::setFilterColor("blue");
				break;
			case autonomousType::RedDown:
				printOnController("Auton: RedDown");
				printf("RedDown\n");
				botintake::setFilterColor("blue");
				break;
			case autonomousType::BlueUp:
				printOnController("Auton: BlueUp");
				printf("BlueUp\n");
				botintake::setFilterColor("red");
				break;
			case autonomousType::BlueDown:
				printOnController("Auton: BlueDown");
				printf("BlueDown\n");
				botintake::setFilterColor("red");
				break;
			case autonomousType::RedUpSafe:
				printOnController("Auton: RedUp SF");
				printf("RedUp Safe\n");
				botintake::setFilterColor("blue");
				break;
			case autonomousType::RedDownSafe:
				printOnController("Auton: RedDown SF");
				printf("RedDown Safe\n");
				botintake::setFilterColor("blue");
				break;
			case autonomousType::BlueUpSafe:
				printOnController("Auton: BlueUp SF");
				printf("BlueUp Safe\n");
				botintake::setFilterColor("red");
				break;
			case autonomousType::BlueDownSafe:
				printOnController("Auton: BlueDown SF");
				printf("BlueDown Safe\n");
				botintake::setFilterColor("red");
				break;
			case autonomousType::AutonSkills:
				printOnController("Auton: Skills");
				printf("AuSk\n");
				botintake::setFilterColor("blue");
				break;
			case autonomousType::DrivingSkills:
				printOnController("Driving Skills");
				printf("DrSk\n");
				botintake::setFilterColor("blue");
				break;
			default:
				printOnController("Auton: None");
				printf("None\n");
				botintake::setFilterColor("none");
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
				break;
			case autonomousType::BlueUpSafe:
				break;
			case autonomousType::BlueDownSafe:
				break;
			case autonomousType::AutonSkills:
				autonpaths::runAutonSkills();
				break;
			case autonomousType::AllianceWallStake:
				autonpaths::runAllianceWallStake();
				break;
			case autonomousType::Test:
				autonpaths::autonTest();
			default:
				break;
		}
	}
}
