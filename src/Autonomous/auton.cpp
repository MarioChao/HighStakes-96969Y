#include "Autonomous/auton.h"

#include "Autonomous/autonPaths.h"
#include "Mechanics/botIntake.h"
#include "Mechanics/botIntake2.h"
#include "Mechanics/botDrive.h"
#include "Utilities/debugFunctions.h"
#include "main.h"

namespace {
using namespace auton;

bool userRunningAutonomous = false;
bool runningAutonUponStart = false;

bool runningState = false;

bool canChangeAutonState = true;

// autonomousType auton_runType = autonomousType::DrivingSkills;
autonomousType auton_runType = autonomousType::Test;
// autonomousType auton_runType = autonomousType::AutonSkillsLong;
// autonomousType auton_runType = autonomousType::BlueSoloAWP;
// autonomousType auton_runType = autonomousType::OdometryRadiusTest;
int auton_allianceId;

std::string autonFilterOutColor = "";
}

namespace auton {
void setAutonRunType(int allianceId, autonomousType autonType) {
	// Validation
	if (isRunning()) return;
	if (!canChangeAuton()) return;

	// Switch run type
	switch (autonType) {
		case autonomousType::RedUp:
			autonpaths::storeProfiles_redUp();
			debug::printOnController("Auton: RedUp");
			printf("RedUp\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::RedDown:
			autonpaths::storeProfiles_redDown();
			debug::printOnController("Auton: RedDown");
			printf("RedDown\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::BlueUp:
			autonpaths::storeProfiles_blueUp();
			debug::printOnController("Auton: BlueUp");
			printf("BlueUp\n");
			autonFilterOutColor = "red";
			break;
		case autonomousType::BlueDown:
			autonpaths::storeProfiles_blueDown();
			debug::printOnController("Auton: BlueDown");
			printf("BlueDown\n");
			autonFilterOutColor = "red";
			break;
		case autonomousType::RedUpSafe:
			autonpaths::storeProfiles_redUpSafe();
			debug::printOnController("Auton: RedUp SF");
			printf("RedUp Safe\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::RedDownSafe:
			autonpaths::storeProfiles_redDownSafe();
			debug::printOnController("Auton: RedDown SF");
			printf("RedDown Safe\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::BlueUpSafe:
			autonpaths::storeProfiles_blueUpSafe();
			debug::printOnController("Auton: BlueUp SF");
			printf("BlueUp Safe\n");
			autonFilterOutColor = "red";
			break;
		case autonomousType::BlueDownSafe:
			autonpaths::storeProfiles_blueDownSafe();
			debug::printOnController("Auton: BlueDown SF");
			printf("BlueDown Safe\n");
			autonFilterOutColor = "red";
			break;

		case autonomousType::RedSoloAWP:
			autonpaths::storeProfiles_redSoloAWP();
			debug::printOnController("Auton: Red SoloAWP");
			printf("Red SoloAWP\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::BlueSoloAWP:
			autonpaths::storeProfiles_blueSoloAWP();
			debug::printOnController("Auton: Blue SoloAWP");
			printf("Blue SoloAWP\n");
			autonFilterOutColor = "red";
			break;

		case autonomousType::AutonSkills:
			autonpaths::storeProfiles_skills();
			debug::printOnController("Auton: Skills");
			printf("AuSk\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::DrivingRunAutonSkills:
			autonpaths::storeProfiles_skills();
			debug::printOnController("Driving -> Auton");
			printf("Dr->AuSk\n");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::DrivingSkills:
			debug::printOnController("Driving Skills");
			printf("DrSk\n");
			autonFilterOutColor = "blue";
			break;

		case autonomousType::LoveShape:
			autonpaths::storeProfiles_loveShape();
			debug::printOnController("Love Shape");
			printf("LoveShape\n");
			autonFilterOutColor = "none";
			break;
		case autonomousType::FieldTour:
			autonpaths::storeProfiles_fieldTour();
			debug::printOnController("Field Tour");
			printf("Field Tour\n");
			autonFilterOutColor = "none";
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
	runningState = true;

	// Set config
	userRunningAutonomous = false;
	botintake::setFilterOutColor(autonFilterOutColor);
	botintake2::setFilterOutColor(autonFilterOutColor);
	botdrive::setMaxDeltaVolt(2.0);
	
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
			autonpaths::runAutonRedDownSafe();
			break;
		case autonomousType::BlueUpSafe:
			autonpaths::runAutonBlueUpSafe();
			break;
		case autonomousType::BlueDownSafe:
			autonpaths::runAutonBlueDown();
			break;
		case autonomousType::RedSoloAWP:
			autonpaths::runRedSoloAWP();
			break;
		case autonomousType::BlueSoloAWP:
			autonpaths::runBlueSoloAWP();
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

	runningState = false;
}

bool isRunning() {
	return runningState;
}

bool canChangeAuton() {
	return canChangeAutonState;
}

void setCanChangeAuton(bool state) {
	canChangeAutonState = state;
}
}
