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

#include "Utilities/fieldInfo.h"
#include "Utilities/debugFunctions.h"

#include "Videos/video-main.h"

#include "AutonUtilities/linegular.h"
#include "AutonUtilities/ramseteController.h"
#include "Simulation/robotSimulator.h"
#include "Utilities/generalUtility.h"


// ---------- Variables ----------

competition Competition;

double motSpeedRpm, motAimSpeedRpm = 0;

int intakePart = 1;

timer drivingTimer;

Odometry mainOdometry;

// Test functions

void test1() {
	RamseteController ramsete;
	RobotSimulator simulator;
	simulator.position = Vector3(0, 0, 0);
	simulator.angularPosition = genutil::toRadians(90.0);
	Linegular lg2(5 * field::tileLengthIn, 5 * field::tileLengthIn, 0);
	while (1) {
		Linegular lg1(simulator.position.x, simulator.position.y, genutil::toDegrees(simulator.angularPosition));
		std::pair<double, double> lrVelocity = ramsete.getLeftRightVelocity_pct(lg1, lg2);
		double scaleFactorLR = genutil::getScaleFactor(1200.0, {lrVelocity.first, lrVelocity.second});
		lrVelocity.first *= scaleFactorLR;
		lrVelocity.second *= scaleFactorLR;

		double velocity = (lrVelocity.first + lrVelocity.second) / 2;
		double angularVelocity = (lrVelocity.second - lrVelocity.first) / 2;
		double scaleFactorAV = genutil::getScaleFactor(genutil::toRadians(180.0), {angularVelocity});
		angularVelocity *= scaleFactorAV;

		double lookAngle = simulator.angularPosition;
		printf("POS: %.3f, %.3f, ANG: %.3f\n", simulator.position.x, simulator.position.y, genutil::toDegrees(lookAngle));
		printf("LR: %.3f, %.3f\n", lrVelocity.first, lrVelocity.second);
		simulator.velocity = Vector3(velocity * cos(lookAngle), velocity * sin(lookAngle), 0);
		simulator.angularVelocity = angularVelocity;
		simulator.updatePhysics();
		wait(20, msec);
	}
}

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

	/* Testing Start */
	test1();
	/* Testing End */

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...

	// Odometry
	mainOdometry.addPositionSensor2D(-90, []() {return LookRotation.position(rev);}, 1, 2, 0);
	mainOdometry.addPositionSensor2D(0, []() {return RightEncoder.position(rev);}, 1, 2.75, 0);
	mainOdometry.addInertialSensor(InertialSensor, -3.276, 3.651);
	mainOdometry.setPositionFactor(1.0 / field::tileLengthIn);
	task odometryTask([]() -> int {
		mainOdometry.setPosition(0, 0);
		mainOdometry.setLookAngle(0);
		mainOdometry.start();
		while (true) {
			mainOdometry.odometryFrame();
			// printf("test: %.3f %.3f\n", mainOdometry.getX(), mainOdometry.getY());
			wait(20, msec);
		}
	});

	// Tasks
	controls::startThreads();
	// odometry::startThreads();
	task rum([]() -> int { preautonControllerThread(); return 1; });

	// Brake-types
	controls::preauton();

	// Sensors
	runPreauton();

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
		video::switchVideoState(1);
		return 1;
	});

	// ..........................................................................
	auton::runAutonomous();


	// ..........................................................................

	printf("Time spent: %.3f s\n", benchmark.value());
}

/// @brief A function for testing autonomous directly in usercontrol.
void userRunAutonomous() {
	// Wait until sensors are initialized
	task::sleep(1500);
	while (!isPreautonFinished()) {
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

	// Prevent main from exiting with an infinite loop.
	while (true) {
		wait(100, msec);
	}
}
