#pragma once

#include <vector>
#include <memory>


namespace pas1_lib {
namespace planning {
namespace segments {


// ---------- Enumeration ----------

enum SplineType {
	Cubic_Bezier,
	Cubic_Hermite,
	CatmullRom,
	Cubic_B_Spline,
	undefined,
};


// ---------- Class ----------

class SegmentBase {
public:
	virtual std::vector<double> getPositionAtT(double t);
	virtual std::vector<double> getFirstPrimeAtT(double t);
	virtual std::vector<double> getSecondPrimeAtT(double t);

	virtual std::vector<std::vector<double>> getControlPoints();

	virtual SplineType getSplineType();

	virtual std::shared_ptr<SegmentBase> getReversed();


	double knot_parameter_alpha;
};


}
}
}
