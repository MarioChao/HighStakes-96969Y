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

const double pathFollowDelay_seconds = 0.20;

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

	// Trajectory graph
	testTrajectoryPlan = splineProfile->trajectoryPlan;
	trajectoryTestTimer.reset();

	// Reset PID
	autonSettings.distanceError_tiles_to_velocity_pct_pid.resetErrorToZero();
	autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.resetErrorToZero();
	autonSettings.angleError_degrees_to_velocity_pct_pid.resetErrorToZero();

	// Reset slew
	autonSettings.linearAcceleration_pctPerSec_slew.reset();
	autonSettings.angularAcceleration_pctPerSec_slew.reset();

	// Reset patience
	autonSettings.distanceError_tiles_patience.reset();

	// Reset timer
	_pathTimer.reset();

	// Get spline info
	double totalDistance_tiles = splineProfile->curveSampler.getDistanceRange().second;
	double totalTime_seconds = splineProfile->trajectoryPlan.getTotalTime();

	// Print info
	printf("Spline %.3f tiles %.3f sec\n", totalDistance_tiles, totalTime_seconds);

	// Follow path
	while (true) {
		/* ---------- End conditions ---------- */

		// Get time
		double traj_time = _pathTimer.time(seconds);

		// Check profile ended
		if (traj_time >= totalTime_seconds + pathFollowDelay_seconds) {
			printf("Profile ended\n");
			break;
		}

		// Check settled
		if (
			autonSettings.distanceError_tiles_to_velocity_pct_pid.isSettled()
			&& autonSettings.angleError_degrees_to_velocity_pct_pid.isSettled()
		) {
			printf("Settled\n");
			break;
		}

		// Check exhausted
		if (
			autonSettings.distanceError_tiles_patience.isExhausted()
			&& autonSettings.angleError_degrees_to_velocity_pct_pid.isSettled()
		) {
			printf("Exhausted\n");
			break;
		}


		/* ---------- Trajectory ---------- */

		// Get trajectory motion
		std::pair<double, std::vector<double>> motion = splineProfile->trajectoryPlan.getMotionAtTime(traj_time);
		double traj_distance = motion.first;
		double traj_velocity = motion.second[0];
		double traj_tvalue = splineProfile->curveSampler.distanceToParam(traj_distance);
		// double traj_curvature = splineProfile->spline.getCurvatureAt(traj_tvalue);
		double traj_curvature = splineProfile->trajectoryPlan.getCurvatureAtDistance(traj_distance);
		double traj_angularVelocity = traj_velocity * traj_curvature;

		// Get robot and target linegular
		Linegular robotLg = chassis->getLookPose();
		Linegular targetLg = splineProfile->spline.getLinegularAt(traj_tvalue, splineProfile->willReverse);

		/* Overall error */

		// Update distance remaining
		double total_distanceError = totalDistance_tiles - traj_distance;
		double pose_distanceError = (targetLg - robotLg).getXYMagnitude();
		printf("TDE: %.3f PDE: %.3f AE: %.3f\n", total_distanceError, pose_distanceError, (targetLg - robotLg).getThetaPolarAngle_degrees());
		_pathFollowDistanceRemaining_tiles = std::fabs(total_distanceError + pose_distanceError);
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(total_distanceError + pose_distanceError);

		// Update angle error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError((targetLg - robotLg).getThetaPolarAngle_degrees());

		// Update error patience
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(total_distanceError + pose_distanceError));

		/* Pose control */

		// Get desired robot motion (linear and angular)
		std::pair<double, double> linegularVelocity = ramseteController.getLinegularVelocity(robotLg, targetLg, traj_velocity, traj_angularVelocity);

		// Convert velocity units
		double linearVelocity_tilesPerSec = linegularVelocity.first;
		double angularVelocity_pct = linegularVelocity.second * botInfo.trackWidth_tiles / 2 * botInfo.tilesPerSecond_to_pct;


		/* ---------- Linear ---------- */

		/* Feedforward + feedback */

		// Motion feedforward
		autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.computeFromMotion(linearVelocity_tilesPerSec, 0);

		// Velocity feedback
		double currentVelocity_tilesPerSec = chassis->getLookVelocity();
		autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.computeFromError(linearVelocity_tilesPerSec - currentVelocity_tilesPerSec);

		// Combined
		bool useS = currentVelocity_tilesPerSec < 0.02;
		double forwardVelocity_volt = autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.getValue(useS);
		double feedbackVelocity_volt = autonSettings.fb_velocityError_tilesPerSec_to_volt_pid.getValue();
		double linearVelocity_pct = aespa_lib::genutil::voltToPct(forwardVelocity_volt + feedbackVelocity_volt);


		/* ---------- Combined ---------- */

		// Scale velocity overshoot
		double leftVelocity_pct = linearVelocity_pct - angularVelocity_pct;
		double rightVelocity_pct = linearVelocity_pct + angularVelocity_pct;
		double scaleFactor = aespa_lib::genutil::getScaleFactor(100.0, { leftVelocity_pct, rightVelocity_pct });
		leftVelocity_pct *= scaleFactor;
		rightVelocity_pct *= scaleFactor;
		linearVelocity_pct = (leftVelocity_pct + rightVelocity_pct) / 2.0;
		angularVelocity_pct = (rightVelocity_pct - leftVelocity_pct) / 2.0;

		// Slew
		autonSettings.linearAcceleration_pctPerSec_slew.computeFromTarget(linearVelocity_pct);
		autonSettings.angularAcceleration_pctPerSec_slew.computeFromTarget(angularVelocity_pct);
		linearVelocity_pct = autonSettings.linearAcceleration_pctPerSec_slew.getValue();
		angularVelocity_pct = autonSettings.angularAcceleration_pctPerSec_slew.getValue();

		// Drive
		chassis->control_local2d(0, linearVelocity_pct, angularVelocity_pct);
		// printf("ACT XY: %.3f, %.3f TAR XY: %.3f, %.3f\n", robotLg.getX(), robotLg.getY(), targetLg.getX(), targetLg.getY());

		// Wait
		wait(10, msec);
	}
	Linegular lg = chassis->getLookPose();
	printf("X: %.3f, Y: %.3f\n", lg.getX(), lg.getY());

	// Stop
	chassis->stopMotors(coast);

	// Settled
	_pathFollowDistanceRemaining_tiles = -1;
	_isPathFollowCompleted = true;
}
}
