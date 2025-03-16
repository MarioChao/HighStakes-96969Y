#include "Pas1-Lib/Planning/Segments/segment-base.h"


namespace pas1_lib {
namespace planning {
namespace segments {


std::vector<double> SegmentBase::getPositionAtT(double t) {
	return {};
}

std::vector<double> SegmentBase::getFirstPrimeAtT(double t) {
	return {};
}

std::vector<double> SegmentBase::getSecondPrimeAtT(double t) {
	return {};
}

std::vector<std::vector<double>> SegmentBase::getControlPoints() {
	return {};
}

SplineType SegmentBase::getSplineType() {
	return SplineType::undefined;
}

std::shared_ptr<SegmentBase> SegmentBase::getReversed() {
	return std::shared_ptr<SegmentBase>();
}


}
}
}
