#include "Autonomous/autonFunctions.h"

#include "Pas1-Lib/Auton/Pose-Controllers/ramseteController.h"
#include "AutonUtilities/linegular.h"

#include "Pas1-Lib/Planning/Splines/uniformCubicSpline.h"
#include "Pas1-Lib/Planning/Splines/curveSampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectoryPlanner.h"

#include "Mechanics/botDrive.h"

#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Simulation/robotSimulator.h"

#include "global-vars.h"
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
		_pathFollowDistanceRemaining_tiles = 10;
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

			// Get total distance
			double totalDistance_tiles = _curveSampler.getDistanceRange().second;

			// Simulator initial
			if (useSimulator) {
				// Linegular lg = _splinePath.getLinegularAt(0, _reverseHeading);
				// robotSimulator.position = Vector3(lg.getX(), lg.getY());
				// robotSimulator.angularPosition = lg.getThetaPolarAngle_radians();
			}

			// Follow path
			while (true) {
				// Get time
				double traj_time = _splinePathTimer.value();

				// Exit when path completed
				if (traj_time > _trajectoryPlan.getTotalTime() + _pathFollowDelay_seconds) {
					_pathFollowCompleted = true;
					_pathFollowDistanceRemaining_tiles = 0;
					break;
				}

				// Get trajectory motion
				std::vector<double> motion = _trajectoryPlan.getMotionAtTime(traj_time);
				double traj_distance = motion[0];
				double traj_velocity = motion[1];
				double traj_tvalue = _curveSampler.distanceToParam(traj_distance);
				double traj_angularVelocity = traj_velocity * _splinePath.getCurvatureAt(traj_tvalue);

				// Update distance remaining
				_pathFollowDistanceRemaining_tiles = totalDistance_tiles - traj_distance;

				// Get robot and target linegular
				Linegular robotLg = mainOdometry.getLookLinegular();
				Linegular targetLg = _splinePath.getLinegularAt(traj_tvalue, _reverseHeading);

				if (useSimulator) {
					robotLg = Linegular(robotSimulator.position.x, robotSimulator.position.y, aespa_lib::genutil::toDegrees(robotSimulator.angularPosition));
				}

				// Get desired robot motion (linear and angular)
				std::pair<double, double> linegularVelocity = robotController.getLinegularVelocity(robotLg, targetLg, traj_velocity, traj_angularVelocity);

				// Drive
				if (!useSimulator) {
					// Convert linear velocity units
					linegularVelocity.first *= _pathToPctFactor;

					botdrive::driveLinegularVelocity(linegularVelocity.first, linegularVelocity.second);
					// printf("Lin: %.3f, ang: %.3f\n", linegularVelocity.first, linegularVelocity.second);
				} else {
					// robotSimulator.position = Vector3(targetLg.getX(), targetLg.getY());
					// robotSimulator.angularPosition = targetLg.getThetaPolarAngle_radians();
					robotSimulator.setForwardVelocity(linegularVelocity.first);
					robotSimulator.angularVelocity = linegularVelocity.second;
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
	double _pathToPctFactor = botinfo::tilesPerSecond_to_pct;
	bool _pathFollowStarted;
	bool _pathFollowCompleted;
	double _pathFollowDistanceRemaining_tiles;
	double _pathFollowDelay_seconds = 0.010;
}
