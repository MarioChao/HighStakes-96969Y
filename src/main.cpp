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
#include "GraphUtilities/trajectoryPlanner.h"


// ---------- Variables ----------

competition Competition;

double motSpeedRpm, motAimSpeedRpm = 0;

int intakePart = 1;

bool isArmPneumatic = false;

timer drivingTimer;

Odometry mainOdometry;

RobotSimulator robotSimulator;

TrajectoryPlanner testTrajectoryPlan;

timer trajectoryTestTimer;

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

	// Preprocess trajectory plan
	double dist = splineSampler1.getDistanceRange().second;
	testTrajectoryPlan._onInit(dist);
	// printf("Spline distance: %.3f\n", dist);
	testTrajectoryPlan.addDesiredMotionConstraints(0, 2.2, 2, 2);
	testTrajectoryPlan.addDesiredMotionConstraints(2, 1.5, 2, 2);
	testTrajectoryPlan.addDesiredMotionConstraints(5, 3, 2, 2);
	testTrajectoryPlan.addDesiredMotionConstraints(8, 2.5, 2, 2);
	testTrajectoryPlan.calculateMotion();

	// Simulator splines
	UniformCubicSpline &spline = spline1;
	CurveSampler &splineSampler = splineSampler1;

	// Direction
	bool isReversed = false;

	// Set initial position
	std::vector<double> pos, vel;
	pos = spline.getPositionAtT(0);
	vel = spline.getVelocityAtT(0);
	// robotSimulator.position = Vector3(pos[0], pos[1], 0);
	// robotSimulator.angularPosition = spline.getLinegularAt(0, isReversed).getTheta_radians();
	ramsete.setDirection(isReversed);
	robotSimulator.setDistance(0);

	// Overwrite splines
	UniformCubicSpline loveSpline = UniformCubicSpline()
		.attachSegment(CubicSplineSegment(cspline::CatmullRom, {
			{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}
		}))
		.extendPoint({1.54, 1.8})
		.extendPoint({2.06, 1.95})
		.extendPoint({2.49, 1.34})
		.extendPoint({1.54, 0.48})
		.extendPoint({0.48, 0.05});
	CurveSampler loveSplineSampler = CurveSampler(loveSpline).calculateByResolution(loveSpline.getTRange().second * 7);
	TrajectoryPlanner loveSplineTrajectoryPlan = TrajectoryPlanner(loveSplineSampler.getDistanceRange().second)
		.addDesiredMotionConstraints(0, 2, 2, 2)
		.addDesiredMotionConstraints(1.2, 1.0, 2, 2)
		.addDesiredMotionConstraints(1.8, 0.5, 2, 2)
		.addDesiredMotionConstraints(3.2, 1.0, 2, 2)
		.addDesiredMotionConstraints(3.8, 2, 2, 2)
		.calculateMotion();
	// printf("Spline distance: %.3f\n", loveSplineSampler.getDistanceRange().second);
	spline = loveSpline;
	splineSampler = loveSplineSampler;
	testTrajectoryPlan = loveSplineTrajectoryPlan;

	// Start route after 1 second
	task::sleep(1000);
	robotSimulator.resetTimer();
	trajectoryTestTimer.reset();

	int id = 0;
	while (1) {
		// Get time
		// double s = trajectoryTestTimer.value() * 1;
		double time = trajectoryTestTimer.value();
		std::vector<double> motion = testTrajectoryPlan.getMotionAtTime(time);
		double s = motion[0];
		double v = motion[1];
		double t = splineSampler.distanceToParam(s);
		// printf("t: %.3f\n", t);
		if (time > testTrajectoryPlan.getTotalTime()) {
			break;
			if (id == 0) {
				spline = splineR1;
				splineSampler = splineSamplerR1;
				isReversed = false;
				ramsete.setDirection(isReversed);
				robotSimulator.setDistance(0);
				trajectoryTestTimer.reset();
			}
			id++;
			wait(20, msec);
			continue;
		}
		// double t = trajectoryTestTimer.value() * 0.5;
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
		std::pair<double, double> lrVelocity = ramsete.getLeftRightVelocity_pct(lg1, lg2, v);
		double scaleFactorLR = genutil::getScaleFactor(3.0, {lrVelocity.first, lrVelocity.second});
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
	mainOdometry.addPositionSensor2D(-90, []() {return LookRotation.position(rev);}, 1, 2, 0.0197);
	mainOdometry.addPositionSensor2D(0, []() {return RightEncoder.position(rev);}, 1, 2.75, 0.4235);
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
	// test1();
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
