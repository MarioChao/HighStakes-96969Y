#pragma once

#include "Aespa-Lib/Karina-Data-Structures/matrix.h"
#include "Pas1-Lib/Planning/Segments/segment-base.h"

#include <utility>
#include <vector>
#include <algorithm>


namespace pas1_lib {
namespace planning {
namespace segments {


// ---------- Constants ----------

namespace characteristic_matrix {
extern aespa_lib::datas::Matrix Cubic_Hermite;
}

namespace storing_matrix {
extern aespa_lib::datas::Matrix Cubic_Bezier;
extern aespa_lib::datas::Matrix Cubic_Hermite;
extern aespa_lib::datas::Matrix CatmullRom;
extern aespa_lib::datas::Matrix Cubic_B_Spline;
}

void setMatrices();


// ---------- Class ----------

class CubicSplineSegment : public SegmentBase {
public:
	/// @brief Constructs a CubicSplineSegment.
	/// @param splineType The type of cubic spline.
	/// @param points The control points for the spline type.
	/// @param knot_parameter_alpha (Not done) Slightly modifies the tangent of Catmull Rom splines.
	CubicSplineSegment(
		SplineType splineType, std::vector<std::vector<double>> points,
		double knot_parameter_alpha = 0
	);
	CubicSplineSegment();

	void setSplineType(SplineType splineType);
	void setPoints(std::vector<std::vector<double>> points);

	SplineType getSplineType() override;
	std::vector<std::vector<double>> getControlPoints() override;

	aespa_lib::datas::Matrix &getCharacteristicMatrix();
	aespa_lib::datas::Matrix &getStoringMatrix();

	std::vector<double> getPositionAtT(double t) override;
	std::vector<double> getFirstPrimeAtT(double t) override;
	std::vector<double> getSecondPrimeAtT(double t) override;

	std::shared_ptr<SegmentBase> getReversed() override;

private:
	SplineType splineType;

	std::vector<std::vector<double>> control_points;
	std::vector<std::vector<double>> stored_points;
};


}
}
}
