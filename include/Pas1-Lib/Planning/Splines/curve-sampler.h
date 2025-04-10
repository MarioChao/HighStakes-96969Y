#pragma once

// Name inspired from https://github.com/FreyaHolmer/Mathfs/blob/master/Runtime/Splines/UniformCurveSampler.cs

#include "Pas1-Lib/Planning/Splines/spline-curve.h"
#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include <vector>
#include <memory>


namespace pas1_lib {
namespace planning {
namespace splines {


// ---------- Struct ----------

struct CurveParam {
	CurveParam(double t, double distance);

	double t;
	double distance;
};


// ---------- Class ----------

class CurveSampler {
public:
	CurveSampler(SplineCurve spline);
	CurveSampler();

	void reset();

	// Configure spline
	void setSpline(std::shared_ptr<SplineCurve> spline);
	std::vector<double> _getCurvePosition(double t);
	std::vector<double> _getCurveFirstPrime(double t);

	// Preprocess the spline to enable sampling
	CurveSampler &calculateByResolution(int resolution = 30);

	// Spline data
	std::pair<double, double> getTRange();
	std::pair<double, double> getDistanceRange();
	SplineCurve &getSpline();
	aespa_lib::datas::Linegular getLinegularAtDistance(double distance, bool reverseHeading = false);

	// Sampling
	double paramToDistance(double t);
	double distanceToParam(double distance);

	std::vector<double> integerParamsToDistances();

private:
	std::vector<CurveParam> t_cumulativeDistances;
	std::shared_ptr<SplineCurve> spline;
};


}
}
}
