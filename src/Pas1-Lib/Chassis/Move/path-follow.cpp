#include "Pas1-Lib/Chassis/Move/path-follow.h"

#include "Pas1-Lib/Auton/Pose-Controls/ramsete.h"

#include "global-vars.h"


namespace {
using aespa_lib::datas::Linegular;
using pas1_lib::planning::profiles::SplineProfile;
using pas1_lib::auton::control_loops::PIDController;
using pas1_lib::auton::control_loops::SlewController;
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

	// Wheel controllers & slew
	PIDController fb_leftVelocity_tilesPerSec_to_volt_pid(autonSettings.fb_velocityError_tilesPerSec_to_volt_pid);
	PIDController fb_rightVelocity_tilesPerSec_to_volt_pid(autonSettings.fb_velocityError_tilesPerSec_to_volt_pid);
	fb_leftVelocity_tilesPerSec_to_volt_pid.resetErrorToZero();
	fb_rightVelocity_tilesPerSec_to_volt_pid.resetErrorToZero();
	SlewController leftAcceleration_pctPerSec_slew(autonSettings.linearAcceleration_pctPerSec_slew);
	SlewController rightAcceleration_pctPerSec_slew(autonSettings.linearAcceleration_pctPerSec_slew);
	leftAcceleration_pctPerSec_slew.reset();
	rightAcceleration_pctPerSec_slew.reset();

	// Reset PID
	autonSettings.distanceError_tiles_to_velocity_pct_pid.resetErrorToZero();
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

	// Debug info
	double maxPoseError = 0;

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
		double traj_motion_distance = motion.first;
		double traj_abs_distance = std::fabs(traj_motion_distance);
		double traj_velocity = motion.second[0];
		double traj_acceleration = motion.second[1];
		double traj_tvalue = splineProfile->curveSampler.distanceToParam(traj_abs_distance);
		double traj_curvature = splineProfile->spline.getCurvatureAt(traj_tvalue);
		double traj_angularVelocity = std::fabs(traj_velocity) * traj_curvature;

		// Get robot and target linegular
		Linegular robotLg = chassis->getLookPose();
		Linegular targetLg = splineProfile->spline.getLinegularAt(traj_tvalue, splineProfile->willReverse);

		/* Overall error */

		// Update distance remaining
		double total_distanceError = totalDistance_tiles - traj_abs_distance;
		double pose_distanceError = (targetLg - robotLg).getXYMagnitude();
		if (pose_distanceError > maxPoseError) {
			maxPoseError = pose_distanceError;
			// printf("TDE: %.3f PDE: %.3f AE: %.3f\n", total_distanceError, maxPoseError, (targetLg - robotLg).getRotation().polarDeg());
			// printf("TDE: %.3f PDE: %.3f AE: %.3f\n", total_distanceError, pose_distanceError, (targetLg - robotLg).getRotation().polarDeg());
		}
		_pathFollowDistanceRemaining_tiles = std::fabs(total_distanceError + pose_distanceError);
		autonSettings.distanceError_tiles_to_velocity_pct_pid.computeFromError(total_distanceError + pose_distanceError);

		// Update angle error
		autonSettings.angleError_degrees_to_velocity_pct_pid.computeFromError((targetLg - robotLg).getRotation().polarDeg());

		// Update error patience
		autonSettings.distanceError_tiles_patience.computePatience(std::fabs(total_distanceError + pose_distanceError));

		
		/* ---------- Pose control ---------- */

		// Get desired robot motion (linear and angular)
		std::pair<double, double> linegularVelocity = ramseteController.getLinegularVelocity(robotLg, targetLg, traj_velocity, traj_angularVelocity);

		// Convert velocity units
		double linearVelocity_tilesPerSec = linegularVelocity.first;
		double angularVelocity_tilesPerSec = linegularVelocity.second * botInfo.trackWidth_tiles / 2;

		// Convert to left & right velocity
		double desiredLeftVelocity_tilesPerSec = linearVelocity_tilesPerSec - angularVelocity_tilesPerSec;
		double desiredRightVelocity_tilesPerSec = linearVelocity_tilesPerSec + angularVelocity_tilesPerSec;
		double scaleFactor = aespa_lib::genutil::getScaleFactor(botInfo.maxVel_tilesPerSec, { desiredLeftVelocity_tilesPerSec, desiredRightVelocity_tilesPerSec });
		desiredLeftVelocity_tilesPerSec *= scaleFactor;
		desiredRightVelocity_tilesPerSec *= scaleFactor;

		// Command
		// chassis->control_local2d(0, traj_velocity * botInfo.tilesPerSecond_to_pct, traj_angularVelocity * botInfo.trackWidth_tiles / 2 * botInfo.tilesPerSecond_to_pct);
		// chassis->control_local2d(0, linearVelocity_tilesPerSec * botInfo.tilesPerSecond_to_pct, angularVelocity_tilesPerSec * botInfo.tilesPerSecond_to_pct);
		// wait(10, msec);
		// continue;


		/* ---------- Velocity control ---------- */

		/* Feedforward + feedback */

		// Current wheel state
		double currentLeftVelocity_tilesPerSec = chassis->getLeftVelocity();
		double currentRightVelocity_tilesPerSec = chassis->getRightVelocity();

		// Left wheel
		double leftVelocity_volt = 0;
		leftVelocity_volt = autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.calculateDiscrete(currentLeftVelocity_tilesPerSec, desiredLeftVelocity_tilesPerSec);
		fb_leftVelocity_tilesPerSec_to_volt_pid.computeFromError(desiredLeftVelocity_tilesPerSec - currentLeftVelocity_tilesPerSec);
		leftVelocity_volt += fb_leftVelocity_tilesPerSec_to_volt_pid.getValue();

		// Right wheel
		double rightVelocity_volt = 0;
		rightVelocity_volt = autonSettings.ff_velocity_tilesPerSec_to_volt_feedforward.calculateDiscrete(currentRightVelocity_tilesPerSec, desiredRightVelocity_tilesPerSec);
		fb_rightVelocity_tilesPerSec_to_volt_pid.computeFromError(desiredRightVelocity_tilesPerSec - currentRightVelocity_tilesPerSec);
		rightVelocity_volt += fb_rightVelocity_tilesPerSec_to_volt_pid.getValue();
		
		/* Slew */
		
		// To volt
		double leftVelocity_pct = aespa_lib::genutil::voltToPct(leftVelocity_volt);
		double rightVelocity_pct = aespa_lib::genutil::voltToPct(rightVelocity_volt);
		
		// Slew
		leftAcceleration_pctPerSec_slew.computeFromTarget(leftVelocity_pct);
		rightAcceleration_pctPerSec_slew.computeFromTarget(rightVelocity_pct);
		leftVelocity_pct = leftAcceleration_pctPerSec_slew.getValue();
		rightVelocity_pct = rightAcceleration_pctPerSec_slew.getValue();
		printf("ERR_LR, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", traj_time, desiredLeftVelocity_tilesPerSec, currentLeftVelocity_tilesPerSec, leftVelocity_pct / botInfo.tilesPerSecond_to_pct, chassis->commanded_leftMotor_volt, desiredRightVelocity_tilesPerSec, currentRightVelocity_tilesPerSec, rightVelocity_pct / botInfo.tilesPerSecond_to_pct, chassis->commanded_rightMotor_volt);

		// Drive
		chassis->control_differential(leftVelocity_pct, rightVelocity_pct);
		// printf("ACT XY: %.3f, %.3f TAR XY: %.3f, %.3f\n", robotLg.getX(), robotLg.getY(), targetLg.getX(), targetLg.getY());

		// Wait
		wait(10, msec);
	}

	Linegular lg = chassis->getLookPose();
	printf("X: %.3f, Y: %.3f\n", lg.getX(), lg.getY());
	printf("Max PDE: %.3f\n", maxPoseError);

	// Stop
	chassis->stopMotors(coast);

	// Settled
	_pathFollowDistanceRemaining_tiles = -1;
	_isPathFollowCompleted = true;
}
}
