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
#include "Autonomous/autonPaths.h"

#include "Controller/controls.h"

#include "Mechanics/botArm.h"
#include "Mechanics/botIntake.h"
#include "Sensors/ringOptical.h"
#include "Sensors/inertial-s.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/debugFunctions.h"

#include "Cosmetics/Videos/video-main.h"
#include "Cosmetics/LedLight/led-main.h"
#include "Cosmetics/Simulation/robotSimulator.h"

#include "Autonomous/autonValues.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Pas1-Lib/Planning/Segments/cubic-spline.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectoryPlanner_old.h"


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

	printf("maxV: %.3f\n", botInfo.maxVel_tilesPerSec);
	printf("???: %.3f\n", botInfo.maxAccel_tilesPerSec2 * 0.2);
	// printf("tps2%%: %.3f\n", botInfo.tilesPerSecond_to_pct);

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
		mainOdometry.setPosition_scaled(3, 3);
		mainOdometry.setLookAngle_field(0);
		mainOdometry.start();
		while (true) {
			mainOdometry.odometryFrame();
			// printf("test, %.3f, %.3f\n", mainOdometry.getX_scaled(), mainOdometry.getY_scaled());
			wait(5, msec);
		}
	});//*/

	// Set auton
	waitUntil(preauton::isBufferFinished());
	auton::setCanChangeAuton(true);
	auton::showAutonRunType();

	// Check motor
	if (LeftMotors.temperature(celsius) == 21 && RightMotors.temperature(celsius) == 21) {
		mainUseSimulator = true;
		printf("Using simulator\n");
	} else {
		printf("Not using simulator\n");
	}

	// Simulator
	if (mainUseSimulator) {
		robotSimulator.position = Vector3(1, 1);
		robotSimulator.angularPosition = aespa_lib::genutil::toRadians(90);
		task simulatorTask([]() -> int {
			while (true) {
				// Set simulation physics
				bool useDifferentialVolt = true;
				if (useDifferentialVolt) {
					// Get actual chassis motion
					double left_volt = robotChassis.commanded_leftMotor_volt;
					double right_volt = robotChassis.commanded_rightMotor_volt;
					robotSimulator.setForwardDifferentialVoltage(
						left_volt, right_volt,
						botInfo.maxVel_tilesPerSec / 12.0, 0.13,
						botInfo.trackWidth_tiles
					);
				} else {
					double linear_volt = (robotChassis.commanded_leftMotor_volt + robotChassis.commanded_rightMotor_volt) / 2;
					double angular_volt = (robotChassis.commanded_rightMotor_volt - robotChassis.commanded_leftMotor_volt) / 2;
					double forwardVelocity_tilesPerSec = linear_volt / 12 * botInfo.maxVel_tilesPerSec;
					double angularVelocity_radiansPerSec = angular_volt / 12 * botInfo.maxVel_tilesPerSec / (botInfo.trackWidth_tiles / 2);
					// double alpha = 0.5;
					// double newForwardVelocity = (1 - alpha) * robotSimulator.getForwardVelocity() + alpha * forwardVelocity_tilesPerSec;
					// double newAngularVelocity = (1 - alpha) * robotSimulator.angularVelocity + alpha * angularVelocity_radiansPerSec;
					robotSimulator.setForwardDifferentialMotion(
						forwardVelocity_tilesPerSec, angularVelocity_radiansPerSec,
						botInfo.maxVel_tilesPerSec, botInfo.maxAccel_tilesPerSec2, botInfo.trackWidth_tiles
					);
				}

				// Update simulation physics
				robotSimulator.constrainMotion(botInfo.maxVel_tilesPerSec, botInfo.trackWidth_tiles);
				robotSimulator.updatePhysics();
				robotSimulator.updateDistance();

				if (mainUseSimulator) {
					// Update actual chassis position & velocity
					aespa_lib::datas::Linegular robotPose = robotChassis.getLookPose();
					robotPose.setPosition(robotSimulator.position.x, robotSimulator.position.y);
					robotPose.setRotation(aespa_lib::genutil::toDegrees(robotSimulator.angularPosition));
					robotChassis.setLookPose(robotPose);
					double linearVelocity = robotSimulator.getForwardVelocity();
					double angularVelocity = robotSimulator.angularVelocity;
					double leftVelocity = linearVelocity - angularVelocity * botInfo.trackWidth_tiles / 2;
					double rightVelocity = linearVelocity + angularVelocity * botInfo.trackWidth_tiles / 2;
					robotChassis.overwriteLookVelocity = { true, linearVelocity };
					robotChassis.overwriteLeftRightVelocity = { true, {leftVelocity, rightVelocity} };
				} else {
					robotChassis.overwriteLookVelocity = { false, 0 };
					robotChassis.overwriteLeftRightVelocity = { false, {0, 0} };
				}

				wait(5, msec);
			}
		});
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
	// Switch to a random video
	task switchVideo([]() -> int {
		// video::switchVideoState(1);
		auton::setCanChangeAuton(false);
		return 1;
	});
	controls::resetStates();

	// ..........................................................................
	auton::runAutonomous();


	// ..........................................................................
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
		auton::setAutonRunType(0, auton::autonomousType::AutonSkills);
		autonomous();
	}

	// Keybinds
	controls::setUpKeybinds();
	video::keybindVideos();

	// Reset
	controls::resetStates();

	// User control code here, inside the loop
	double v = 0;
	std::vector<double> velocities = { 0 };
	while (1) {
		if (false) {
			LeftRightMotors.spin(forward, v, volt);

			double velocity = LeftRightMotors.velocity(pct) / botInfo.tilesPerSecond_to_pct;
			if (velocities.size() > 10) velocities = { velocity };
			else velocities.push_back(velocity);
			double avgV = aespa_lib::genutil::getAverage(velocities);
			if (fabs(v) > 0) printf("volt: %.3f, vel: %.3f\n", v, avgV);

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
