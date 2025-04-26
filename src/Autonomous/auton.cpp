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

// autonomousType auton_runType = autonomousType::None;
autonomousType auton_runType = autonomousType::DrivingSkills;
// autonomousType auton_runType = autonomousType::Test;
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
			autonFilterOutColor = "blue";
			break;
		case autonomousType::RedUpSafe:
			autonpaths::storeProfiles_redUpSafe();
			debug::printOnController("Auton: RedUp SF");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::BlueUp:
			autonpaths::storeProfiles_blueUp();
			debug::printOnController("Auton: BlueUp");
			autonFilterOutColor = "red";
			break;
		case autonomousType::BlueUpSafe:
			autonpaths::storeProfiles_blueUpSafe();
			debug::printOnController("Auton: BlueUp SF");
			autonFilterOutColor = "red";
			break;
		case autonomousType::RedDown:
			autonpaths::storeProfiles_redDown();
			debug::printOnController("Auton: RedDown");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::RedDownSafe:
			autonpaths::storeProfiles_redDownSafe();
			debug::printOnController("Auton: RedDown SF");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::RedDownLBRush:
			autonpaths::storeProfiles_redDownLBRush();
			debug::printOnController("Auton: RedDown LB");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::BlueDown:
			autonpaths::storeProfiles_blueDown();
			debug::printOnController("Auton: BlueDown");
			autonFilterOutColor = "red";
			break;
		case autonomousType::BlueDownSafe:
			autonpaths::storeProfiles_blueDownSafe();
			debug::printOnController("Auton: BlueDown SF");
			autonFilterOutColor = "red";
			break;
		case autonomousType::BlueDownLBRush:
			autonpaths::storeProfiles_blueDownLBRush();
			debug::printOnController("Auton: BlueDown LB");
			autonFilterOutColor = "red";
			break;
		case autonomousType::RedSoloAWP:
			autonpaths::storeProfiles_redSoloAWP();
			debug::printOnController("Auton: Red SoloAWP");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::BlueSoloAWP:
			autonpaths::storeProfiles_blueSoloAWP();
			debug::printOnController("Auton: Blue SoloAWP");
			autonFilterOutColor = "red";
			break;

		case autonomousType::AutonSkills:
			autonpaths::storeProfiles_skills();
			debug::printOnController("Auton: Skills");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::DrivingRunAutonSkills:
			autonpaths::storeProfiles_skills();
			debug::printOnController("Driving -> Auton");
			autonFilterOutColor = "blue";
			break;
		case autonomousType::DrivingSkills:
			debug::printOnController("Driving Skills");
			autonFilterOutColor = "blue";
			break;

		case autonomousType::LoveShape:
			autonpaths::storeProfiles_loveShape();
			debug::printOnController("Love Shape");
			autonFilterOutColor = "none";
			break;
		case autonomousType::FieldTour:
			autonpaths::storeProfiles_fieldTour();
			debug::printOnController("Field Tour");
			autonFilterOutColor = "none";
			break;
		case autonomousType::Test:
			autonpaths::storeProfiles_test();
			debug::printOnController("Auton Test");
			autonFilterOutColor = "none";
			break;
		case autonomousType::RushTest:
			// autonpaths::storeProfiles_test();
			debug::printOnController("Rush Test");
			autonFilterOutColor = "none";
			break;
		default:
			debug::printOnController("Auton: None");
			autonFilterOutColor = "none";
			break;
	}
	auton_runType = autonType;
	auton_allianceId = allianceId;
	printf("%s\n", getAutonMode_string().c_str());
}

void showAutonRunType() {
	setAutonRunType(auton_allianceId, auton_runType);
}


autonomousType getAutonRunType() {
	return auton_runType;
}

std::string getAutonMode_string() {
	switch (auton_runType) {
		case autonomousType::AutonSkills: return "AuSk";
		case autonomousType::DrivingSkills: return "DrSk";
		case autonomousType::DrivingRunAutonSkills: return "Dr->AuSk";
		case autonomousType::RedUp: return "R-up";
		case autonomousType::BlueUp: return "B-up";
		case autonomousType::RedDown: return "R-dn";
		case autonomousType::BlueDown: return "B-dn";
		case autonomousType::RedDownLBRush: return "R-dn-lb";
		case autonomousType::BlueDownLBRush: return "B-dn-lb";
		case autonomousType::RedSoloAWP: return "R-sl";
		case autonomousType::BlueSoloAWP: return "B-sl";

		case autonomousType::LoveShape: return "T-Love";
		case autonomousType::FieldTour: return "T-Field";
		case autonomousType::Test: return "T-Auton";
		case autonomousType::RushTest: return "T-Rush";
		case autonomousType::OdometryRadiusTest: return "T-Odom";
		default: return "none";
	}
}


bool isUserRunningAuton() {
	return userRunningAutonomous;
}

bool isRunningAutonUponStart() {
	return runningAutonUponStart;
}


void runAutonomous() {
	// Start autonomous
	timer benchmark;

	printf("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n");
	printf("Auton time!\n");
	runningState = true;

	// Set config
	userRunningAutonomous = false;
	botintake::setAntiJam(true);
	botintake::setFilterOutColor(autonFilterOutColor);
	botintake2::setFilterOutColor(autonFilterOutColor);
	
	// Run auton
	switch (auton_runType) {
		case autonomousType::RedUp:
			autonpaths::runAutonRedUp();
			break;
		case autonomousType::RedUpSafe:
			autonpaths::runAutonRedUpSafe();
			break;
		case autonomousType::BlueUp:
			autonpaths::runAutonBlueUp();
			break;
		case autonomousType::BlueUpSafe:
			autonpaths::runAutonBlueUpSafe();
			break;
		case autonomousType::RedDown:
			autonpaths::runAutonRedDown();
			break;
		case autonomousType::RedDownSafe:
			autonpaths::runAutonRedDownSafe();
			break;
		case autonomousType::RedDownLBRush:
			autonpaths::runAutonRedDownLBRush();
			break;
		case autonomousType::BlueDown:
			autonpaths::runAutonBlueDown();
			break;
		case autonomousType::BlueDownSafe:
			autonpaths::runAutonBlueDownSafe();
			break;
		case autonomousType::BlueDownLBRush:
			autonpaths::runAutonBlueDownLBRush();
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
		case autonomousType::RushTest:
			autonpaths::runRushTest();
			break;
		case autonomousType::LoveShape:
			autonpaths::runLoveShape();
			break;
		case autonomousType::FieldTour:
			autonpaths::runFieldTour();
			break;
		case autonomousType::Test:
			autonpaths::runAutonTest();
			break;
		case autonomousType::OdometryRadiusTest:
			autonpaths::odometryRadiusTest();
			break;
		default:
			break;
	}

	runningState = false;

	printf("----- Time spent: %.3f sec -----\n", benchmark.value());
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
