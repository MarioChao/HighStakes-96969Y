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
	using autonvals::defaultMoveTilesErrorRange, autonvals::defaultMoveWithInchesErrorRange, autonvals::defaultTurnAngleErrorRange;

	void setRotation(double rotation);

	/* PID differential */

	void turnToAngle(double rotation, double rotateCenterOffsetIn = 0, double errorRange = defaultTurnAngleErrorRange, double runTimeout = 3);
	void turnToAngleVelocity(double rotation, double maxVelocityPct, double rotateCenterOffsetIn = 0, double errorRange = defaultTurnAngleErrorRange, double runTimeout = 3);

	void driveDistanceTiles(double distanceTiles, double maxVelocityPct = 100, double errorRange = defaultMoveTilesErrorRange, double runTimeout = 3);
	void driveAndTurnDistanceTiles(double distanceTiles, double targetRotation, double maxVelocityPct = 100, double maxTurnVelocityPct = 100, double errorRange = defaultMoveTilesErrorRange, double runTimeout = 3);
	void driveAndTurnDistanceWithInches(double distanceInches, double targetRotation, double maxVelocityPct = 100, double maxTurnVelocityPct = 100, double errorRange = defaultMoveWithInchesErrorRange, double runTimeout = 3);

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


	/* Main mechanics */

	void setIntakeState(int state, double delaySec = 0);
	void setIntakeTopState(int, double = 0);
	void setIntakeBottomState(int, double = 0);
	void setIntakeToArm(int);

	void setIntakeFilterOutColor(std::string colorText);

	void setArmHangState(int, double = 0);
	void setArmStage(int, double = 0);

	bool isArmResetted();


	/* Legacy wings */

	void setFrontWingsState(bool state, double delaySec = 0);
	void setLeftWingState(bool state, double delaySec = 0);
	void setRightWingState(bool state, double delaySec = 0);
	void setBackWingsState(bool state, double delaySec = 0);

	void setGoalClampState(bool state, double delaySec = 0);
	void setIntakeLiftState(bool state);
}
