#include "Pas1-Lib/Chassis/Move/path-follow.h"

#include "Pas1-Lib/Auton/Pose-Controls/ramsete.h"

#include "global-vars.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::planning::profiles::SplineProfile;
using pas1_lib::chassis::base::Differential;
using pas1_lib::chassis::settings::BotInfo;
using pas1_lib::chassis::settings::AutonSettings;
using namespace pas1_lib::chassis::move::follow;

void runFollowPath();

const double pathFollowDelay_seconds = 0.010;

// Controller
pas1_lib::auton::pose_controllers::RamseteController ramseteController;

SplineProfile *_splineProfile;
Differential *_diff_chassis;
}


namespace pas1_lib {
namespace chassis {
namespace move {
namespace follow {


void followPath(Differential &chassis, followPath_params params, bool async) {
	_splineProfile = params.splineProfile;
	_diff_chassis = &chassis;

	// Profile validation
	if (_splineProfile == nullptr) {
		_isPathFollowCompleted = true;
		return;
	}

	_isPathFollowCompleted = false;

	if (async) {
		task asyncDrive([]() -> int {
			runFollowPath();
			return 1;
		});
	} else {
		runFollowPath();
	}
}

timer _pathTimer;
double _pathFollowDistanceRemaining_tiles;
bool _isPathFollowCompleted;


}
}
}
}


namespace {
void runFollowPath() {
	// Get global variables
	SplineProfile *splineProfile = _splineProfile;
	Differential *chassis = _diff_chassis;
	BotInfo &botInfo = chassis->botInfo;
	AutonSettings &autonSettings = chassis->autonSettings;
	ramseteController.setDirection(splineProfile->willReverse);

	// Reset timer
	_pathTimer.reset();

	// Trajectory graph
	testTrajectoryPlan = splineProfile->trajectoryPlan;
	trajectoryTestTimer.reset();

	// Get spline info
	double totalDistance_tiles = splineProfile->curveSampler.getDistanceRange().second;
	double totalTime_seconds = splineProfile->trajectoryPlan.getTotalTime();

	// Print info
	printf("Spline %.3f tiles %.3f sec\n", totalDistance_tiles, totalTime_seconds);

	// Follow path
	while (true) {
		// Get time
		double traj_time = _pathTimer.time(seconds);

		// Exit when path completed
		if (traj_time > totalTime_seconds + pathFollowDelay_seconds) {
			break;
		}

		// Get trajectory motion
		// std::pair<double, std::vector<double>> motion = _trajectoryPlan.getMotionAtTime(traj_time);
		std::pair<double, std::vector<double>> motion = splineProfile->trajectoryPlan.getMotionAtTime(traj_time);
		double traj_distance = motion.first;
		double traj_velocity = motion.second[0];
		// double traj_tvalue = _curveSampler.distanceToParam(traj_distance);
		double traj_tvalue = splineProfile->curveSampler.distanceToParam(traj_distance);
		// double traj_curvature = _splinePath.getCurvatureAt(traj_tvalue);
		// double traj_curvature = _trajectoryPlan.getCurvatureAtDistance(traj_distance);
		// double traj_curvature = splineProfile->spline.getCurvatureAt(traj_tvalue);
		double traj_curvature = splineProfile->trajectoryPlan.getCurvatureAtDistance(traj_distance);
		double traj_angularVelocity = traj_velocity * traj_curvature;
		
		// Update distance remaining
		_pathFollowDistanceRemaining_tiles = std::fabs(totalDistance_tiles - traj_distance);

		// Get robot and target linegular
		// Linegular robotLg = mainOdometry.getLookPose_scaled();
		// Linegular targetLg = _splinePath.getLinegularAt(traj_tvalue, _reverseHeading);
		Linegular robotLg = chassis->getLookPose();
		Linegular targetLg = splineProfile->spline.getLinegularAt(traj_tvalue, splineProfile->willReverse);


		// Get desired robot motion (linear and angular)
		// std::pair<double, double> linegularVelocity = robotController.getLinegularVelocity(robotLg, targetLg, traj_velocity, traj_angularVelocity);
		std::pair<double, double> linegularVelocity = ramseteController.getLinegularVelocity(robotLg, targetLg, traj_velocity, traj_angularVelocity);

		// Convert velocity units
		// double linearVelocity_pct = linegularVelocity.first * botInfo.tilesPerSecond_to_pct;
		double angularVelocity_pct = linegularVelocity.second * botInfo.trackWidth_tiles / 2 * botInfo.tilesPerSecond_to_pct;

		autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.computeFromMotion(linegularVelocity.first, 0);
		autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.computeFromError(linegularVelocity.first - robotChassis.getLookVelocity());
		double forwardVelocity_volt = autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.getValue(false);
		double feedbackVelocity_volt = autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.getValue();
		double linearVelocity_pct = aespa_lib::genutil::voltToPct(forwardVelocity_volt + feedbackVelocity_volt);

		// Drive
		// botdrive::driveLinegularVelocity(linegularVelocity.first, linegularVelocity.second);
		chassis->control_local2d(0, linearVelocity_pct, angularVelocity_pct);
		// printf("ACT XY: %.3f, %.3f TAR XY: %.3f, %.3f\n", robotLg.getX(), robotLg.getY(), targetLg.getX(), targetLg.getY());

		// Wait
		wait(10, msec);
	}

	// Settled
	_pathFollowDistanceRemaining_tiles = -1;
	_isPathFollowCompleted = true;
}
}
