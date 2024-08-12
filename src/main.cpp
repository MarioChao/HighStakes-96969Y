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

#include "Controller/controls.h"

#include "Utilities/fieldInfo.h"
#include "Utilities/debugFunctions.h"

#include "Videos/brainVideos.h"

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

// ---------- Variables ----------
double botX = 5 * field::tileLengthIn * 2.54;
double botY = 0.5 * field::tileLengthIn * 2.54;
double botAngle = 180;
double motSpeedRpm, motAimSpeedRpm = 0;

int playingVideoId = 0;

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

    // Tasks
    controls::startThreads();
    task rum([] () -> int { preautonControllerThread(); return 1; });

    // Brake-types
    controls::preauton();

    // Sensors
    runPreauton();

    // Debug
    showAutonRunType();
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
    task switchVideo([] () -> int {
        srand(Brain.Timer.systemHighResolution());
        switchVideoState(rand() % 3 + 1);
        return 1;
    });

    // ..........................................................................
    runAutonomous();
    
    
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
    // User autonomous
    if (isUserRunningAuton()) {
        userRunAutonomous();
    }

    // Keybinds
    controls::setUpKeybinds();
    keybindVideos();

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
