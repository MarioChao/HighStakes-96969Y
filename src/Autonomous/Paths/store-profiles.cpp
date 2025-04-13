#include "Autonomous/autonPaths.h"


namespace {
using namespace autonpaths::pathbuild;

using pas1_lib::planning::splines::SplineCurve;
using pas1_lib::planning::splines::CurveSampler;
using namespace pas1_lib::planning::segments;
}

namespace autonpaths {

void storeProfiles_test() {
	storeNewSplineProfile("test",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			// kite
			{ {1.5, -0.94}, {1.5, 0.5}, {1.0, 1.15}, {1.5, 1.73}, {2.0, 1.15}, {1.5, 0.5}, {1.5, -0.94} }
			// mirrored kite
			// { {1.5, -0.94}, {1.5, 0.5}, {2.0, 1.15}, {1.5, 1.73}, {1.0, 1.15}, {1.5, 0.5}, {1.5, -0.94} }
			// high curvature
			// { {2.54, 0.49}, {1.54, 0.47}, {0.47, 0.94}, {1.32, 1.59}, {1.54, 0.47}, {1.5, -0.46} }
			// "vex" word
			// {{0.54, 2.81}, {0.47, 5.38}, {1.2, 4.35}, {1.87, 5.45}, {2.5, 4.91}, {3.52, 4.96}, {3.02, 5.47}, {2.57, 4.47}, {3.89, 4.47}, {5.01, 5.4}, {4.03, 4.94}, {4.13, 5.38}, {4.96, 4.46}, {5.48, 2.84}}
		), false
	);
}

void storeProfiles_redUp() {
	storeNewSplineProfile("ru grab 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {-0.64, 2.63}, {0.53, 3.29}, {2, 4}, {3.55, 4.52} }
		), true,
		{
			new PolygonRegionConstraint(Polygon2D({ {1.5, 4}, {2, 3.5}, {2.5, 4}, {2, 4.5} }), botInfo.maxVel_tilesPerSec * 0.3)
		}
	);
	storeNewSplineProfile("ru ring 1-1a",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {0.38, 3.14}, {2, 4}, {2.47, 4.6}, {2.52, 5.2}, {2.54, 8.0} }
		), false
	);
	storeNewSplineProfile("ru ring 1-1b",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {2.52, 6.19}, {2.52, 5.2}, {2.25, 4.4}, {0.68, 2.37} }
		), true
	);
	storeNewSplineProfile("ru ring 2-1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {4.86, 5.49}, {2.01, 5}, {0.22, 5.74}, {-0.39, 6.51} }
		), false
	);
	storeNewSplineProfile("ru ring 3-1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {1.02, 6.15}, {1.02, 5}, {1, 3}, {1, 2.03} }
		), false,
		{
			new PolygonRegionConstraint(Polygon2D({ {0.5, 3}, {1, 2.5}, {1.5, 3}, {1, 3.5} }), botInfo.maxVel_tilesPerSec * 0.3)
		}
	);
}
void storeProfiles_redUpSafe() {}
void storeProfiles_blueUp() {}
void storeProfiles_blueUpSafe() {}
void storeProfiles_redDown() {
	storeNewSplineProfile("rd grab 2",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {2.01, 0.07}, {2.0, 1}, {2.01, 1.99}, {2.01, 3} }
		), true
	);
	storeNewSplineProfile("rd ring 2-1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {3.23, 2.63}, {2.01, 2.01}, {0.97, 1.54}, {-0.25, 0.94} }
		), false
	);
	storeNewSplineProfile("rd grab 3",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {-0.31, 1.5}, {1, 1.54}, {1.9, 1.2}, {2.01, 0.45}, {2.01, -0.48} }
		), true
	);
	storeNewSplineProfile("rd ring 3-1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {3.18, 0.29}, {2.01, 0.51}, {0.57, 0.39}, {-0.48, -0.47} }
		), false
	);
	storeNewSplineProfile("rd ladder 1a",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {-0.49, -0.05}, {0.54, 0.44}, {2.26, 0.71}, {4.25, 0.08} }
		), true
	);
	storeNewSplineProfile("rd ladder 1b",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {2.61, -0.52}, {2.3, 0.58}, {2.41, 2.4}, {4.05, 3.75} }
		), false
	);
}
void storeProfiles_redDownSafe() {}
void storeProfiles_blueDown() {}
void storeProfiles_blueDownSafe() {}
void storeProfiles_redSoloAWP() {
	storeNewSplineProfile("rsa grab 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {-0.64, 2.63}, {0.53, 3.29}, {2, 4}, {3.55, 4.52} }
		), true,
		{
			new PolygonRegionConstraint(Polygon2D({ {1.5, 4}, {2, 3.5}, {2.5, 4}, {2, 4.5} }), botInfo.maxVel_tilesPerSec * 0.3)
		}
	);
	storeNewSplineProfile("rsa ladder",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {2.34, -0.36}, {2, 1}, {2.3, 2.2}, {4.6, 3.93} }
		), false
	);
}
void storeProfiles_blueSoloAWP() {
	storeNewSplineProfile("bsa grab 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {6-(-0.64), 2.63}, {6-(0.53), 3.29}, {6-(2), 4}, {6-(3.55), 4.52} }
		), true,
		{
			new PolygonRegionConstraint(Polygon2D({ {3.5, 4}, {4, 3.5}, {4.5, 4}, {4, 4.5} }), botInfo.maxVel_tilesPerSec * 0.3)
		}
	);
	storeNewSplineProfile("bsa ladder",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {6-(2.34), -0.36}, {6-(2), 1}, {6-(2.3), 2.2}, {6-(4.6), 3.93} }
		), false
	);
}

void storeProfiles_skills() {
	// Skills
	storeNewSplineProfile("skills 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {0.9, 2.33}, {2.01, 1.02}, {2.98, 0.53}, {4.07, 1.08}, {5.06, 2.31} }
		), false
	);
	storeNewSplineProfile("skills ladder",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {0.13, 3.81}, {0.51, 2.99}, {1.38, 1.87}, {2.24, 2.35}, {3.23, 3.42} }
		), false
	);
}

void storeProfiles_loveShape() {
	storeNewSplineProfile("big curvature 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {1.59, -0.42}, {1.52, 0.5}, {1.49, 0.81}, {0.48, 1}, {1.55, 1.02}, {2.51, 1}, {1.57, 1.28}, {1.53, 1.81}, {1.53, 2.79} }
		), false
	);
	storeNewSplineProfile("big curvature 1br",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{ {1.53, 2.79}, {1.53, 1.81}, {1.57, 1.28}, {2.51, 1}, {1.55, 1.02}, {0.48, 1}, {1.49, 0.81}, {1.52, 0.5}, {1.59, -0.42} }
		), true
	);

	storeNewSplineProfile("love 1",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
				{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05}
			}
		), false
	);

	storeNewSplineProfile("love 1r",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{2.62, 0.09}, {1.52, 0.49}, {0.67, 1.35}, {1.03, 1.97}, {1.54, 1.8},
				{2.06, 1.95}, {2.49, 1.34}, {1.54, 0.48}, {0.48, 0.05}
			}
		), true
	);

	storeNewSplineProfile("love 2",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{4.07, -0.01}, {3, 0.55}, {1.99, 1.99}, {0.92, 4.03}, {1.5, 5.2},
				{3.02, 4.68}, {4.52, 5.2}, {5.08, 4.01}, {4.03, 2.03}, {3.02, 0.57},
				{1.97, -0.07}
			}
		), false
	);
}

void storeProfiles_fieldTour() {
	storeNewSplineProfile("field tour",
		SplineCurve::fromAutoTangent_cubicSpline(CatmullRom,
			{
				{-0.02, -0.07}, {1.5, 0.5}, {2.4, 1.55}, {0.97, 2.99}, {0.42, 4.03},
				{0.74, 5.28}, {2, 5.54}, {2.01, 3.98}, {3.02, 3}, {4.03, 4.02},
				{3.02, 4.85}, {3.02, 5.51}, {4.39, 5.49}, {4.67, 4.2}, {5.55, 3.07},
				{4.65, 1.77}, {5.49, 0.98}, {4.31, 0.42}, {4.02, 1.33}, {3.15, 1.37},
				{3, 0.48}, {3.02, -0.22}
			}
		), false
	);
}

}
