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
	bool runningAutonUponStart = false;

	autonomousType auton_runType = autonomousType::DrivingSkills;
	// autonomousType auton_runType = autonomousType::AutonSkills;
	// autonomousType auton_runType = autonomousType::BlueSoloAWP;
	// autonomousType auton_runType = autonomousType::OdometryRadiusTest;
	int auton_allianceId;

	std::string autonFilterOutColor = "";
}

namespace auton {
	void setAutonRunType(int allianceId, autonomousType autonType) {
		switch (autonType) {
			case autonomousType::RedUp:
				debug::printOnController("Auton: RedUp");
				printf("RedUp\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::RedDown:
				debug::printOnController("Auton: RedDown");
				printf("RedDown\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::BlueUp:
				debug::printOnController("Auton: BlueUp");
				printf("BlueUp\n");
				autonFilterOutColor = "red";
				break;
			case autonomousType::BlueDown:
				debug::printOnController("Auton: BlueDown");
				printf("BlueDown\n");
				autonFilterOutColor = "red";
				break;
			case autonomousType::RedUpSafe:
				debug::printOnController("Auton: RedUp SF");
				printf("RedUp Safe\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::RedDownSafe:
				debug::printOnController("Auton: RedDown SF");
				printf("RedDown Safe\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::BlueUpSafe:
				debug::printOnController("Auton: BlueUp SF");
				printf("BlueUp Safe\n");
				autonFilterOutColor = "red";
				break;
			case autonomousType::BlueDownSafe:
				debug::printOnController("Auton: BlueDown SF");
				printf("BlueDown Safe\n");
				autonFilterOutColor = "red";
				break;

			case autonomousType::RedSoloAWP:
				debug::printOnController("Auton: Red SoloAWP");
				printf("Red SoloAWP\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::BlueSoloAWP:
				debug::printOnController("Auton: Blue SoloAWP");
				printf("Blue SoloAWP\n");
				autonFilterOutColor = "red";
				break;

			case autonomousType::AutonSkills59:
				debug::printOnController("Auton: Skills 59");
				printf("AuSk 59\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::AutonSkillsNoWallStake:
				debug::printOnController("Auton: Skills No WS");
				printf("AuSk NoWS\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::DrivingRunAutonSkills:
				debug::printOnController("Driving -> Auton");
				printf("Dr->AuSk\n");
				autonFilterOutColor = "blue";
				break;
			case autonomousType::DrivingSkills:
				debug::printOnController("Driving Skills");
				printf("DrSk\n");
				autonFilterOutColor = "blue";
				break;
			default:
				debug::printOnController("Auton: None");
				printf("None\n");
				autonFilterOutColor = "none";
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

	bool isRunningAutonUponStart() {
		return runningAutonUponStart;
	}


	void runAutonomous() {
		printf("Auton time!\n");

		// Set config
		userRunningAutonomous = false;
		botintake::setFilterOutColor(autonFilterOutColor);
		botintake2::setFilterOutColor(autonFilterOutColor);

		// Run auton
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
				autonpaths::runAutonRedDown();
				break;
			case autonomousType::BlueUpSafe:
				autonpaths::runAutonBlueUpSafe();
				break;
			case autonomousType::BlueDownSafe:
				autonpaths::runAutonBlueDownSafe();
				break;
			case autonomousType::RedSoloAWP:
				autonpaths::runRedSoloAWP();
				break;
			case autonomousType::BlueSoloAWP:
				autonpaths::runBlueSoloAWP();
				break;
			case autonomousType::AutonSkills59:
				autonpaths::runAutonSkills59();
				break;
			case autonomousType::AutonSkillsNoWallStake:
				autonpaths::runAutonSkillsNoWallStake();
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
