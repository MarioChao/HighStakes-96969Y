/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       User                                                      */
/*    Created:      6/12/2024, 12:24:28 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "main.h"
#include "global-vars.h"

#include "MatchSequence/preauton.h"
#include "MatchSequence/match-end.h"
#include "Autonomous/auton.h"

#include "Controller/controls.h"

#include "Mechanics/botArm.h"
#include "Mechanics/botIntake.h"
#include "Sensors/ringOptical.h"
#include "Sensors/inertial-s.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"

#include "Videos/video-main.h"
#include "LedLight/led-main.h"

#include "Autonomous/autonValues.h"
#include "Simulation/robotSimulator.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Pas1-Lib/Planning/Splines/uniformCubicSpline.h"
#include "Pas1-Lib/Planning/Splines/curveSampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectoryPlanner.h"


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

	printf("maxV: %.3f\n", botinfo::maxV_tilesPerSec);
	// printf("tps2%%: %.3f\n", botinfo::tilesPerSecond_to_pct);

	// Tasks
	controls::startThreads();
	task rum([]() -> int { preauton::controllerThread(); return 1; });
	match_end::startThread();
	ledlight::startThread();
	ringoptical::startThread();
	inertial_s::startThread();

	// Brake-types
	controls::preauton();

	// Sensors
	preauton::run();

	// Odometry
	// /*
	task odometryTask([]() -> int {
		wait(500, msec);
		mainOdometry.setPosition_scaled(1, 1);
		mainOdometry.setLookAngle(0);
		mainOdometry.start();
		while (true) {
			mainOdometry.odometryFrame();
			Controller1.Screen.setCursor(3, 0);
			// Controller1.Screen.print("Enc val: %.3f\n", RightEncoder.position(rev));
			// printf("test: %.3f %.3f\n", mainOdometry.getX_scaled(), mainOdometry.getY_scaled());
			wait(5, msec);
		}
	});//*/

	// Simulator
	robotSimulator.position = Vector3(1, 1);
	robotSimulator.angularPosition = aespa_lib::genutil::toRadians(90);
	task simulatorTask([]() -> int {
		while (true) {
			robotSimulator.updatePhysics();
			robotSimulator.updateDistance();
			wait(5, msec);
		}
	});

	// Debug
	auton::showAutonRunType();

	// Check motor
	if (LeftMotors.temperature(celsius) == 21 && RightMotors.temperature(celsius) == 21) {
		mainUseSimulator = true;
		printf("Using simulator\n");
	}
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
		// botarm::setResetDefaultStage(2);
		botintake::setFilterOutColor("none");
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
	double v = 0;
	std::vector<double> velocities = {0};
	while (1) {
		if (false) {
			LeftRightMotors.spin(forward, v, volt);

			double velocity = LeftRightMotors.velocity(pct) / 100.0 * botinfo::maxV_tilesPerSec;
			if (velocities.size() > 10) velocities = {velocity};
			else velocities.push_back(velocity);
			double avgV = aespa_lib::genutil::getAverage(velocities);
			if (fabs(v) > 1) printf("volt: %.3f, vel: %.3f\n", v, avgV);

			if (Controller2.ButtonRight.pressing()) {
				v += 0.1;
			}
			if (Controller2.ButtonLeft.pressing()) {
				v -= 0.1;
			}
		} else {
			controls::doControls();
		}

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
