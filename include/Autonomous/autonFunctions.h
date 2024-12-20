#pragma once

#include "Autonomous/autonValues.h"

#include "AutonUtilities/odometry.h"

#include "main.h"


// Forward declaration

class UniformCubicSpline;
class CurveSampler;
class TrajectoryPlanner;


// Namespace

namespace autonfunctions {
	/* General */

	void setRobotRotation(double rotation);

	extern timer _autonTimer;

	/* PID differential */

	void turnToAngle(double rotation, double rotateCenterOffsetIn = 0, double runTimeout = 3);
	void turnToAngleVelocity(double rotation, double maxVelocityPct, double rotateCenterOffsetIn = 0, double runTimeout = 3);

	void driveDistanceTiles(double distanceTiles, double maxVelocityPct = 100, double runTimeout = 3);
	void driveAndTurnDistanceTiles(double distanceTiles, double targetRotation, double maxVelocityPct = 100, double maxTurnVelocityPct = 100, double runTimeout = 3);
	void driveAndTurnDistanceWithInches(double distanceInches, double targetRotation, double maxVelocityPct = 100, double maxTurnVelocityPct = 100, double runTimeout = 3);

	/* PID + Odometry */

	namespace driveturn {
		void async_driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse = false, double maxVelocityPct = 100, double maxTurnVelocityPct = 100, double runTimeout = 3);
		void driveTurnToFace_tiles(double x_tiles, double y_tiles, bool isReverse = false, double maxVelocityPct = 100, double maxTurnVelocityPct = 100, double runTimeout = 3);

		extern double _linearPathDistanceError;
		extern double _targetX, _targetY;
		extern bool _isReverseHeading;
		extern double _maxVelocity_pct, _maxTurnVelocity_pct;
		extern double _runTimeout;
		extern bool _isDriveTurnSettled;
	}

	void turnToFace_tiles(double x_tiles, double y_tiles, bool isReverse = false, double maxTurnVelocity_pct = 100);

	void runLinearPIDPath(std::vector<std::vector<double>> waypoints, double maxVelocity, bool isReverse = false);

	void setDifferentialUseRelativeRotation(bool useRelativeRotation);

	extern bool _useRelativeRotation;


	/* Path following */

	void setSplinePath(UniformCubicSpline &splinePath, TrajectoryPlanner &trajectoryPlan);
	void setSplinePath(UniformCubicSpline &splinePath, TrajectoryPlanner &trajectoryPlan, CurveSampler &curveSampler);
	void setPathToPctFactor(double factor = autonvals::tilesPerSecond_to_pct);
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
