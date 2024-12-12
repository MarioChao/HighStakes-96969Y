#include "Autonomous/autonPaths.h"

namespace autonpaths { namespace pathbuild {
	// Build paths

	const double maxVel = 3.7 * 0.7;
	const double maxAccel = 6;
	const double maxDecel = 6;

	std::vector<UniformCubicSpline> splines;
	std::vector<CurveSampler> splineSamplers;
	std::vector<TrajectoryPlanner> splineTrajectoryPlans;
	std::vector<bool> willReverse;

	int pathIndex;

	void pushNewSpline(UniformCubicSpline spline, bool reverse, double maxVel) {
		CurveSampler splineSampler = CurveSampler(spline)
			.calculateByResolution(spline.getTRange().second * 10);
		TrajectoryPlanner splineTrajectoryPlan = TrajectoryPlanner(splineSampler.getDistanceRange().second)
			.autoSetMotionConstraints(splineSampler, 0.3, maxVel, maxAccel, maxDecel)
			.calculateMotion();
		splines.push_back(spline);
		splineSamplers.push_back(splineSampler);
		splineTrajectoryPlans.push_back(splineTrajectoryPlan);
		willReverse.push_back(reverse);
	}

	void loadSkillsSplines(int section) {
		// Clear
		splines.clear();
		splineSamplers.clear();
		splineTrajectoryPlans.clear();
		willReverse.clear();
		pathIndex = 0;

		UniformCubicSpline spline;

		if (section == 1) {
			// Alliance wall stake to mobile goal
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.64, 2.86}, {0.29, 2.92}, {1, 2.04}, {0.96, 0.02}
			});
			pushNewSpline(spline, true);

			// Redirect 1 ring to arm & score 3 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.25, 1.99}, {1, 2.03}, {1.93, 2.25}, {2.05, 0.98}, {3, 0.57}, {4.01, 1}, {5.02, 1.75}
			});
			pushNewSpline(spline);

			// Prepare to score on neutral wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.08, 1.52}, {4.05, 1.02}, {3.18, 0.7}, {3.02, 1.1}, {2.98, 2.39}
			});
			pushNewSpline(spline, true);

			// Score 2 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{4.54, 0.13}, {2.98, 0.45}, {1.79, 0.98}, {0.57, 1}, {-0.35, 1.02}
			});
			pushNewSpline(spline);
		} else if (section == 2) {
			// Redirect middle ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.05, -0.09}, {0.55, 0.49}, {1.59, 1.67}, {3, 2.98}, {4.21, 4.09}
			});
			pushNewSpline(spline);

			// Store ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{2.03, 3.99}, {3.02, 3}, {4.01, 1.99}, {4.86, 1.18}
			});
			pushNewSpline(spline);

			// Grab mobile goal
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{1.07, 1.5}, {4.01, 2.03}, {5.02, 2.95}, {5, 4.14}
			});
			pushNewSpline(spline, true);

			// Face alliance wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.85, 3}, {5.02, 3}, {4.52, 3}, {3.19, 3}
			});
			pushNewSpline(spline, true);

			// Score 2 rings at top
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.11, 1.9}, {5.26, 2.88}, {4.76, 4.03}, {5.01, 5}, {5, 5.47}, {5, 6.25}
			});
			pushNewSpline(spline);

			// Score 2 rings at bottom
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.25, 5.71}, {5.49, 5}, {4.65, 4.08}, {4.75, 2.09}, {4.99, 1.01}, {5, 0.54}, {5.02, -0.41}
			});
			pushNewSpline(spline);

			// Face alliance wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.16, -0.14}, {5.5, 0.59}, {4.87, 1.4}, {4.55, 2.7}, {5.04, 3.01}, {6.3, 2.91}
			});
			pushNewSpline(spline);
		} else if (section == 3) {
			// Push mobile goal to corner
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.3, 2.05}, {5.43, 3}, {5.66, 5.63}, {5.76, 6.36}
			});
			pushNewSpline(spline, true);
		} else if (section == 4) {
			// Redirect 1 ring and store 1 ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{6.11, 5.85}, {5.48, 5.43}, {3.98, 5}, {2.01, 3.99}, {0.33, 1.36}
			});
			pushNewSpline(spline);

			// Grab goal
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{3.26, 4.01}, {2.03, 4.01}, {1, 3.99}, {-0.5, 4.05}
			});
			pushNewSpline(spline, true);

			// Go to neutral wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.3, 4.07}, {1.05, 4}, {2.01, 4.14}, {2.83, 4.8}, {3.02, 5.51}, {3.04, 6.35}
			});
			pushNewSpline(spline);

			// Score 1 ring
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.85, 5.41}, {3, 5.45}, {4.02, 4.03}, {4.08, 1.01}
			});
			pushNewSpline(spline);

			// Score 3 rings
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{5.63, 2.25}, {3.96, 4.05}, {2.4, 4.93}, {0.49, 5}, {-0.42, 5}
			});
			pushNewSpline(spline);
		} else if (section == 5) {
			// Go to alliance wall stake
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{-0.19, 6.07}, {0.47, 5.53}, {1.3, 3.5}, {0.59, 3.01}, {-0.42, 3}
			});
			pushNewSpline(spline);

			// Climb on ladder
			spline = UniformCubicSpline::fromAutoTangent(cspline::CatmullRom, {
				{0.13, 3.81}, {0.51, 2.99}, {1.38, 1.87}, {2.24, 2.35}, {3.23, 3.42}
			});
			pushNewSpline(spline);
		}
	}

	void runFollowSpline() {
		autonfunctions::setSplinePath(splines[pathIndex], splineTrajectoryPlans[pathIndex], splineSamplers[pathIndex]);
		autonfunctions::followSplinePath(willReverse[pathIndex]);

		pathIndex++;
	}
}}
