#pragma once

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/units.h"
#include "Pas1-Lib/Planning/Segments/segment-base.h"
#include "Pas1-Lib/Planning/Segments/cubic-spline.h"

#include <initializer_list>
#include <vector>
#include <memory>


namespace pas1_lib {
namespace planning {
namespace splines {


// ---------- Class ----------

class SplineCurve {
public:
	SplineCurve(std::vector<std::shared_ptr<segments::SegmentBase>> segments);
	SplineCurve();

	/// @brief Creates a spline with the given points. Only use B-Spline or Catmull-Rom.
	static SplineCurve fromAutoTangent_cubicSpline(
		segments::SplineType splineType, std::vector<std::vector<double>> points
	);

	/// @brief Extends the spline by adding a new segment. Only use for B-Spline or Catmull-Rom.
	SplineCurve &extendPoint_cubicSpline(std::vector<double> &newPoint);

	/// @brief Extends the spline by adding new segments. Only use for B-Spline or Catmull-Rom.
	SplineCurve &extendPoints_cubicSpline(std::vector<std::vector<double>> &newPoints);

	SplineCurve &attachSegment(std::shared_ptr<segments::SegmentBase> newSegment);

	std::vector<std::shared_ptr<segments::SegmentBase>> getSegments();

	segments::SegmentBase &getSegment(int id);

	std::pair<int, double> getSegmentIndex(double t);

	std::vector<double> getPositionAtT(double t);
	std::vector<double> getFirstPrimeAtT(double t);
	std::vector<double> getSecondPrimeAtT(double t);

	aespa_lib::units::PolarAngle getPolarAngleAt(double t);
	double getCurvatureAt(double t);

	std::pair<double, double> getTRange();

	SplineCurve getReversed();

	aespa_lib::datas::Linegular getLinegularAt(double t, bool reverseHeading = false);

private:
	std::vector<std::shared_ptr<segments::SegmentBase>> segments;
};


}
}
}
