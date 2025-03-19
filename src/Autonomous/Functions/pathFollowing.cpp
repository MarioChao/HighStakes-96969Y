#include "Autonomous/autonFunctions.h"

#include "Pas1-Lib/Auton/Pose-Controls/ramsete.h"

#include "Pas1-Lib/Planning/Segments/cubic-spline.h"
#include "Pas1-Lib/Planning/Splines/curve-sampler.h"
#include "Pas1-Lib/Planning/Trajectories/trajectoryPlanner_old.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Mechanics/botDrive.h"

#include "Simulation/robotSimulator.h"

#include "global-vars.h"
#include "main.h"

namespace {

using aespa_lib::datas::Linegular;
using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using pas1_lib::planning::trajectories::TrajectoryPlanner;

// Controller
pas1_lib::auton::pose_controllers::RamseteController robotController;

}


namespace autonfunctions {

void setSplinePath(SplineCurve &splinePath, TrajectoryPlanner &trajectoryPlan) {
	double resolution = splinePath.getTRange().second * 7;
	setSplinePath(splinePath, trajectoryPlan, CurveSampler(splinePath).calculateByResolution(resolution));
}

void setSplinePath(SplineCurve &splinePath, TrajectoryPlanner &trajectoryPlan, CurveSampler &curveSampler) {
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

		// Get spline info
		double totalDistance_tiles = _curveSampler.getDistanceRange().second;
		double totalTime_seconds = _trajectoryPlan.getTotalTime();

		// Print info
		printf("Spline %.3f tiles %.3f sec\n", totalDistance_tiles, totalTime_seconds);

		// Simulator initial
		if (mainUseSimulator) {
			// Linegular lg = _splinePath.getLinegularAt(0, _reverseHeading);
			// robotSimulator.position = Vector3(lg.getX(), lg.getY());
			// robotSimulator.angularPosition = lg.getThetaPolarAngle_radians();
		}

		// Follow path
		while (true) {
			// Get time
			double traj_time = _splinePathTimer.time(seconds);

			// Get trajectory motion
			std::pair<double, std::vector<double>> motion = _trajectoryPlan.getMotionAtTime(traj_time);
			double traj_distance = motion.first;
			double traj_velocity = motion.second[0];
			double traj_tvalue = _curveSampler.distanceToParam(traj_distance);
			double traj_angularVelocity = traj_velocity * _splinePath.getCurvatureAt(traj_tvalue);
			
			// Update distance remaining
			_pathFollowDistanceRemaining_tiles = totalDistance_tiles - traj_distance;

			// Get robot and target linegular
			Linegular robotLg = mainOdometry.getLookPose_scaled();
			Linegular targetLg = _splinePath.getLinegularAt(traj_tvalue, _reverseHeading);

			if (mainUseSimulator) {
				robotLg = Linegular(robotSimulator.position.x, robotSimulator.position.y, aespa_lib::genutil::toDegrees(robotSimulator.angularPosition));
			}

			// Get desired robot motion (linear and angular)
			std::pair<double, double> linegularVelocity = robotController.getLinegularVelocity(robotLg, targetLg, traj_velocity, traj_angularVelocity);

			// Drive
			if (!mainUseSimulator) {
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

			// Exit when path completed
			if (traj_time > totalTime_seconds + _pathFollowDelay_seconds) {
				_pathFollowCompleted = true;
				_pathFollowDistanceRemaining_tiles = 0;
				break;
			}

			// Wait
			wait(10, msec);
		}

		// Return int
		return 1;
	});
}

timer _splinePathTimer;
SplineCurve _splinePath;
TrajectoryPlanner _trajectoryPlan;
CurveSampler _curveSampler;
bool _reverseHeading;
double _pathToPctFactor = botinfo::tilesPerSecond_to_pct;
bool _pathFollowStarted;
bool _pathFollowCompleted;
double _pathFollowDistanceRemaining_tiles;
double _pathFollowDelay_seconds = 0.010;

}
