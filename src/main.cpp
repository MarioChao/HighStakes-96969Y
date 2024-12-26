/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       User                                                      */
/*    Created:      6/12/2024, 12:24:28 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "main.h"
#include "preauton.h"
#include "Autonomous/auton.h"

#include "AutonUtilities/odometry.h"
#include "Controller/controls.h"

#include "Mechanics/botArm.h"
#include "Mechanics/botIntake.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/debugFunctions.h"

#include "Videos/video-main.h"

#include "AutonUtilities/linegular.h"
#include "Simulation/robotSimulator.h"
#include "Utilities/generalUtility.h"

#include "GraphUtilities/matrix.h"
#include "GraphUtilities/uniformCubicSpline.h"
#include "GraphUtilities/curveSampler.h"
#include "GraphUtilities/trajectoryPlanner.h"


// ---------- Variables ----------

competition Competition;

int intakePart = 1;

bool isArmPneumatic = false;

timer drivingTimer;

Odometry mainOdometry;

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
	vexcodeInit();

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...

	// Odometry
	mainOdometry.addPositionSensor2D(-90, []() {return LookRotation.position(rev);}, 1, 2.005, 0);
	mainOdometry.addPositionSensor2D(180, []() {return RightEncoder.position(rev);}, 1, 2.75, -3.5);
	// mainOdometry.addInertialSensor(InertialSensor, -3.2, 2.1);
	// mainOdometry.addInertialSensor(InertialSensor, -2.8, 2.8);
	mainOdometry.addInertialSensor(InertialSensor, -4, 4);
	// mainOdometry.addInertialSensor(InertialSensor, 0, 0);
	mainOdometry.setPositionFactor(1.0 / field::tileLengthIn);
	task odometryTask([]() -> int {
		wait(500, msec);
		mainOdometry.setPosition(1, 1);
		mainOdometry.setLookAngle(0);
		mainOdometry.start();
		while (true) {
			mainOdometry.odometryFrame();
			Controller1.Screen.setCursor(3, 0);
			// Controller1.Screen.print("Enc val: %.3f\n", RightEncoder.position(rev));
			// printf("test: %.3f %.3f\n", mainOdometry.getX(), mainOdometry.getY());
			wait(5, msec);
		}
	});

	// Tasks
	controls::startThreads();
	// odometry::startThreads();
	// preauton::controllerThread();
	task rum([]() -> int { preauton::controllerThread(); return 1; });

	// Brake-types
	controls::preauton();

	// Sensors
	preauton::run();

	// Debug
	auton::showAutonRunType();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
	// Start autonomous
	timer benchmark;

	// Switch to a random video
	task switchVideo([]() -> int {
		// video::switchVideoState(1);
		return 1;
	});
	controls::resetStates();

	// ..........................................................................
	auton::runAutonomous();


	// ..........................................................................

	printf("Time spent: %.3f s\n", benchmark.value());
}

/// @brief A function for testing autonomous directly in usercontrol.
void userRunAutonomous() {
	// Wait until sensors are initialized
	task::sleep(1500);
	while (!preauton::isFinished()) {
		task::sleep(10);
	}

	// userRunAutonomous();
	autonomous();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
	// Timer
	drivingTimer.reset();

	// User autonomous
	if (auton::isUserRunningAuton()) {
		userRunAutonomous();
	}

	// Driving skills
	if (auton::getAutonRunType() == auton::autonomousType::DrivingSkills) {
		botarm::setResetDefaultStage(2);
		botintake::setFilterOutColor("blue");
	} else if (auton::getAutonRunType() == auton::autonomousType::DrivingRunAutonSkills) {
		auton::setAutonRunType(0, auton::autonomousType::AutonSkills59);
		autonomous();
	}

	// Keybinds
	controls::setUpKeybinds();
	video::keybindVideos();

	// Reset
	controls::resetStates();

	// User control code here, inside the loop
	while (1) {
		controls::doControls();

		wait(20, msec);
	}
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
	// Set up callbacks for autonomous and driver control periods.
	Competition.autonomous(autonomous);
	Competition.drivercontrol(usercontrol);

	// Run the pre-autonomous function.
	pre_auton();

	// Start autonomous
	if (auton::isRunningAutonUponStart()) {
		userRunAutonomous();
	}

	// Prevent main from exiting with an infinite loop.
	while (true) {
		wait(100, msec);
	}
}
