#include "Autonomous/autonFunctions.h"

#include "AutonUtilities/ramseteController.h"
#include "AutonUtilities/linegular.h"
#include "AutonUtilities/odometry.h"

#include "GraphUtilities/uniformCubicSpline.h"
#include "GraphUtilities/curveSampler.h"
#include "GraphUtilities/trajectoryPlanner.h"

#include "Mechanics/botDrive.h"

#include "Utilities/generalUtility.h"

#include "Simulation/robotSimulator.h"

#include "main.h"

namespace {
	// Controller
	RamseteController robotController;

	// Simulator
	bool useSimulator = mainUseSimulator;
}

namespace autonfunctions {
	void setSplinePath(UniformCubicSpline &splinePath, TrajectoryPlanner &trajectoryPlan) {
		double resolution = splinePath.getTRange().second * 7;
		setSplinePath(splinePath, trajectoryPlan, CurveSampler(splinePath).calculateByResolution(resolution));
	}

	void setSplinePath(UniformCubicSpline &splinePath, TrajectoryPlanner &trajectoryPlan, CurveSampler &curveSampler) {
		_splinePath = splinePath;
		_trajectoryPlan = trajectoryPlan;
		_curveSampler = curveSampler;
		_pathFollowStarted = false;
		_pathFollowCompleted = false;
		_reverseHeading = false;
	}

	void setPathToPctFactor(double factor) {
		_pathToPctFactor = factor;
	}

	void followSplinePath(bool reverseHeading) {
		// Initialize config
		_pathFollowStarted = true;
		_pathFollowCompleted = false;
		_reverseHeading = reverseHeading;
		robotController.setDirection(reverseHeading);

		// Follow path in thread
		task followPathTask([]() -> int {
			// Reset timer
			_splinePathTimer.reset();

			if (useSimulator) {
				robotSimulator.resetTimer();
			}

			// Follow path
			while (true) {
				// Get time
				double traj_time = _splinePathTimer.value();

				// Exit when path completed
				if (traj_time > _trajectoryPlan.getTotalTime()) {
					_pathFollowCompleted = true;
					break;
				}

				// Get trajectory motion
				std::vector<double> motion = _trajectoryPlan.getMotionAtTime(traj_time);
				double traj_distance = motion[0];
				double traj_velocity = motion[1];
				double traj_tvalue = _curveSampler.distanceToParam(traj_distance);

				// Get robot and target linegular
				Linegular robotLg = mainOdometry.getLookLinegular();
				Linegular targetLg = _splinePath.getLinegularAt(traj_tvalue, _reverseHeading);

				if (useSimulator) {
					robotLg = Linegular(robotSimulator.position.x, robotSimulator.position.y, genutil::toDegrees(robotSimulator.angularPosition));
				}

				// Get robot motion
				std::pair<double, double> leftRightVelocity = robotController.getLeftRightVelocity_pct(robotLg, targetLg, traj_velocity);

				// Get motor percentages
				double leftVelocityPct, rightVelocityPct;
				leftVelocityPct = leftRightVelocity.first * _pathToPctFactor;
				rightVelocityPct = leftRightVelocity.second * _pathToPctFactor;

				// Drive
				if (!useSimulator) {
					botdrive::driveVelocity(leftVelocityPct, rightVelocityPct);
					// botdrive::driveVoltage(genutil::pctToVolt(leftVelocityPct), genutil::pctToVolt(rightVelocityPct), 11);
					// printf("L: %07.3f, R: %07.3f\n", leftVelocityPct, rightVelocityPct);
				} else {
					double velocity = (leftRightVelocity.first + leftRightVelocity.second) / 2;
					double angularVelocity = (leftRightVelocity.second - leftRightVelocity.first) / 2;
					double lookAngle = robotSimulator.angularPosition;
					robotSimulator.velocity = Vector3(velocity * cos(lookAngle), velocity * sin(lookAngle), 0);
					robotSimulator.angularVelocity = angularVelocity;

					// robotSimulator.position = Vector3(targetLg.getX(), targetLg.getY());
					// robotSimulator.angularPosition = targetLg.getTheta_radians();
					robotSimulator.updatePhysics();
					robotSimulator.updateDistance();
				}

				// Wait
				wait(20, msec);
			}

			// Return int
			return 1;
		});
	}

	timer _splinePathTimer;
	UniformCubicSpline _splinePath;
	TrajectoryPlanner _trajectoryPlan;
	CurveSampler _curveSampler;
	bool _reverseHeading;
	double _pathToPctFactor = autonvals::tilesPerSecond_to_pct;
	bool _pathFollowStarted;
	bool _pathFollowCompleted;
}
