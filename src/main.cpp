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

#include "GraphUtilities/matrix.h"
#include "GraphUtilities/uniformCubicSpline.h"
#include "GraphUtilities/curveSampler.h"


// ---------- Variables ----------

competition Competition;

double motSpeedRpm, motAimSpeedRpm = 0;

int intakePart = 1;

bool isArmPneumatic = false;

timer drivingTimer;

Odometry mainOdometry;

RobotSimulator robotSimulator;

// Test functions

void test1() {
	// Initialize controller
	RamseteController ramsete(5.0, 0.8);

	// Create a path
	UniformCubicSpline spline1({
		CubicSplineSegment(cspline::SplineType::B_Spline, {
			{0, 0},
			{0, 3},
			{3, 0},
			{3, 3},
		})
	});
	spline1.extendPoint({3, 6});
	spline1.extendPoint({6, 3});
	spline1.extendPoint({6, 6});
	UniformCubicSpline splineR1 = spline1.getReversed();

	// Preprocess the path
	CurveSampler splineSampler1(spline1);
	splineSampler1.calculateByResolution(30);
	CurveSampler splineSamplerR1(splineR1);
	splineSamplerR1.calculateByResolution(30);

	// Simulator splines
	UniformCubicSpline &spline = spline1;
	CurveSampler &splineSampler = splineSampler1;

	// Direction
	bool isReversed = true;

	// Set initial position
	std::vector<double> pos = spline.getPositionAtT(0);
	std::vector<double> vel = spline.getVelocityAtT(0);
	robotSimulator.position = Vector3(pos[0], pos[1], 0);
	robotSimulator.angularPosition = spline.getLinegularAt(0, isReversed).getTheta_radians();
	ramsete.setDirection(isReversed);
	robotSimulator.setDistance(0);
	// robotSimulator.position = Vector3(0, 0, 0);
	// robotSimulator.angularPosition = genutil::toRadians(90.0);
	// Set goal linegular
	task::sleep(1000);
	robotSimulator.resetTimer();
	timer curveTimer;

	int id = 0;
	while (1) {
		// Get time
		// double s = curveTimer.value() * 1;
		double s = robotSimulator.travelledDistance + 0.2;
		double t = splineSampler.distanceToParam(s);
		// printf("t: %.3f\n", t);
		if (t >= spline.getTRange().second) {
			if (id == 0) {
				spline = splineR1;
				splineSampler = splineSamplerR1;
				isReversed = false;
				ramsete.setDirection(isReversed);
				robotSimulator.setDistance(0);
			}
			id++;
			wait(20, msec);
			continue;
		}
		// double t = curveTimer.value() * 0.5;
		// if (t > 4) {
		// 	break;
		// }

		// Get actual & desired linegular
		Linegular lg1(robotSimulator.position.x, robotSimulator.position.y, genutil::toDegrees(robotSimulator.angularPosition));
		// Linegular lg2(0, 0, 0);
		Linegular lg2 = spline.getLinegularAt(t, isReversed);
		// std::vector<double> pos = spline.getPositionAtT(t);
		// std::vector<double> vel = spline.getVelocityAtT(t);
		// Control
		std::pair<double, double> lrVelocity = ramsete.getLeftRightVelocity_pct(lg1, lg2, 1);
		double scaleFactorLR = genutil::getScaleFactor(50.0, {lrVelocity.first, lrVelocity.second});
		lrVelocity.first *= scaleFactorLR;
		lrVelocity.second *= scaleFactorLR;

		double velocity = (lrVelocity.first + lrVelocity.second) / 2;
		double angularVelocity = (lrVelocity.second - lrVelocity.first) / 2;
		double scaleFactorAV = genutil::getScaleFactor(genutil::toRadians(360.0), {angularVelocity});
		angularVelocity *= scaleFactorAV;

		double lookAngle = robotSimulator.angularPosition;
		// printf("POS: %.3f, %.3f, ANG: %.3f\n", robotSimulator.position.x, robotSimulator.position.y, genutil::toDegrees(lookAngle));
		// printf("LR: %.3f, %.3f, ANG: %7.3f\n", lrVelocity.first, lrVelocity.second, angularVelocity);
		robotSimulator.velocity = Vector3(velocity * cos(lookAngle), velocity * sin(lookAngle), 0);
		robotSimulator.angularVelocity = angularVelocity;

		// Test curve
		// pos = spline.getPositionAtT(t);
		// vel = spline.getVelocityAtT(t);
		// robotSimulator.position = Vector3(pos[0], pos[1]);
		// robotSimulator.angularPosition = atan2(vel[1], vel[0]);

		robotSimulator.updatePhysics();
		robotSimulator.updateDistance();
		wait(20, msec);
	}
}

void test2() {
	// Matrix m1({
	// 	{1, 2},
	// 	{3, 4},
	// });
	// Matrix m2({
	// 	{5, 6},
	// 	{7, 8},
	// });
	// auto m3 = m1;
	// m3 *= 5;
	// printf("%s\n", m1.getString().c_str());
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

	/* Testing Start */
	// test2();
	test1();
	/* Testing End */
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
