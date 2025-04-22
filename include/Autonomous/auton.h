#pragma once

#include <string>


namespace auton {


/* Enums */

enum autonomousType {
	RedUp, RedUpSafe,
	BlueUp, BlueUpSafe,
	RedDown, RedDownSafe, RedDownLBRush,
	BlueDown, BlueDownSafe, BlueDownLBRush,
	RedSoloAWP, BlueSoloAWP,
	AutonSkills,
	DrivingRunAutonSkills, DrivingSkills,
	LoveShape, FieldTour,
	Test, OdometryRadiusTest, RushTest,
	None
};


/* Functions */

/// @brief Set the type of autonomous to run at the start of the match.
/// @param allianceId A number representing the alliance (Red: 1, Blue: 2, Neutral: 0).
/// @param autonType The type of autonomus to run.
void setAutonRunType(int allianceId, autonomousType autonType);

void showAutonRunType();

/// @brief Get the type of autonomous that is selected.
autonomousType getAutonRunType();
std::string getAutonMode_string();

/// @brief Get whether autonomous code should run when user control begins.
bool isUserRunningAuton();

bool isRunningAutonUponStart();

/// @brief Run the autonomous set by setAutonRunType().
void runAutonomous();

void autonSkillsIntro();

/// @brief Check whether an autonomous route is running.
bool isRunning();

bool canChangeAuton();
void setCanChangeAuton(bool state);


}
