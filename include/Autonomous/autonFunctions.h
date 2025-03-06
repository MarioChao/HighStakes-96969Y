#pragma once

#include "Autonomous/autonValues.h"
#include "Utilities/robotInfo.h"

#include "main.h"


// Forward declaration

class UniformCubicSpline;
class CurveSampler;
class TrajectoryPlanner;


// Namespace

namespace autonfunctions {
	/* General */

	void setRobotRotation(double fieldAngle_degrees);

	extern timer _autonTimer;

	void setDifferentialUseRelativeRotation(bool useRelativeRotation);

	extern bool _useRelativeRotation;

	/* PID differential */

	namespace pid_diff {
		/// @brief Turn the robot to face a specified angle.
		/// @param fieldAngle_degrees The target angle to face in degrees.
		/// @param rotateCenterOffset_inches The offset of the center of rotation.
		/// @param runTimeout_sec Maximum seconds the function will run for.
		void turnToAngle(double fieldAngle_degrees, double rotateCenterOffset_inches = 0, double runTimeout_sec = 3);
		void turnToAngleVelocity(double fieldAngle_degrees, double maxVelocity_pct, double rotateCenterOffset_inches = 0, double runTimeout_sec = 3);
	
		void driveDistanceTiles(double distance_tiles, double maxVelocity_pct = 100, double runTimeout_sec = 3);
		void driveAndTurnDistanceTiles(double distance_tiles, double fieldAngle_degrees, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);
		void driveAndTurnDistanceWithInches(double distance_inches, double fieldAngle_degrees, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);

		void async_driveAndTurnDistance_tiles(double distance_tiles, double fieldAngle_degrees, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);
		void async_driveAndTurnDistance_qtInches(double distance_qtInches, double fieldAngle_degrees, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);
		void async_driveAndTurnDistance_qtInches(double distance_qtInches, double fieldAngle_degrees, std::vector<std::pair<double, double>> velocityConstraint_qtInch_pct, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);
		void async_driveAndTurnDistance_inches(double distance_inches, double fieldAngle_degrees, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);
		void async_driveAndTurnDistance_inches(double distance_inches, double fieldAngle_degrees, std::vector<std::pair<double, double>> velocityConstraint_inch_pct, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);

		extern double _driveDistanceError_inches;
		extern bool _isDriveAndTurnSettled;
	}

	/* PID + Odometry */

	namespace driveturn {
		void async_driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse = false, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);
		void driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse = false, double maxVelocity_pct = 100, double maxTurnVelocity_pct = 100, double runTimeout_sec = 3);

		extern double _linearPathDistanceError;
		extern double _targetX, _targetY;
		extern bool _isReverseHeading;
		extern double _maxVelocity_pct, _maxTurnVelocity_pct;
		extern double _runTimeout;
		extern bool _isDriveTurnSettled;
	}

	void turnToFace_tiles(double x_tiles, double y_tiles, bool isReverse = false, double maxTurnVelocity_pct = 100);

	void runLinearPIDPath(std::vector<std::vector<double>> waypoints, double maxVelocity, bool isReverse = false);


	/* Path following */

	void setSplinePath(UniformCubicSpline &splinePath, TrajectoryPlanner &trajectoryPlan);
	void setSplinePath(UniformCubicSpline &splinePath, TrajectoryPlanner &trajectoryPlan, CurveSampler &curveSampler);
	void setPathToPctFactor(double factor = botinfo::tilesPerSecond_to_pct);
	void followSplinePath(bool reverseHeading = false);

	extern timer _splinePathTimer;
	extern UniformCubicSpline _splinePath;
	extern TrajectoryPlanner _trajectoryPlan;
	extern CurveSampler _curveSampler;
	extern bool _reverseHeading;
	extern double _pathToPctFactor;
	extern bool _pathFollowStarted;
	extern bool _pathFollowCompleted;
	extern double _pathFollowDistanceRemaining_tiles;
	extern double _pathFollowDelay_seconds;


	/* Main mechanics */

	// Intake
	void setIntakeState(int state, double delaySec = 0);
	void setIntakeTopState(int, double = 0);
	void setIntakeBottomState(int, double = 0);

	void setIntakeToArm(int, double = 0);
	void setIntakeStoreRing(int, double = 0);

	void setIntakeFilterOutColor(std::string colorText);
	void setIntakeFilterEnabled(bool, double = 0);

	// Clamp
	void setGoalClampState(bool state, double delaySec = 0);
	void setIntakeLiftState(bool state);

	// Arm
	void setArmHangState(int, double = 0);
	void setArmStage(int, double = 0);

	bool isArmResetted();
	void setArmResetDefaultStage(int);

	// Swing
	void setSwingState(int, double = 0);
	void setSwing2State(int, double = 0);


	/* Legacy wings */

	void setFrontWingsState(bool state, double delaySec = 0);
	void setLeftWingState(bool state, double delaySec = 0);
	void setRightWingState(bool state, double delaySec = 0);
	void setBackWingsState(bool state, double delaySec = 0);
}
