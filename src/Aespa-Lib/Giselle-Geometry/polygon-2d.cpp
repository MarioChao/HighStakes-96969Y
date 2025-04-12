#include "Aespa-Lib/Giselle-Geometry/polygon-2d.hpp"

#include "Aespa-Lib/Winter-Utilities/general.h"

// Some more polygon info: https://codeforces.com/blog/entry/48868


namespace {
double epsilon = 1e-9;
}


namespace aespa_lib {
namespace geometry {


Polygon2D::Polygon2D(std::vector<Vector2D> ccwPoints)
	: ccwPoints(ccwPoints) {}

double Polygon2D::getArea() {
	// Shoelace formula & optimized
	int n = (int) ccwPoints.size();
	double area = 0;
	for (int pointId = 0; pointId < n; pointId++) {
		int nextId = (pointId + 1) % n;
		int prevId = (pointId - 1 + n) % n;
		area += ccwPoints[pointId].x * (ccwPoints[nextId].y - ccwPoints[prevId].y);
	}
	area /= 2;
	return area;
}

int Polygon2D::getWindingNumber(Vector2D point) {
	// Winding number / ray casting algorithm
	// Winding number: https://en.wikipedia.org/wiki/Winding_number
	int n = (int) ccwPoints.size();
	int windingNumber = 0;
	for (int pointId = 0; pointId < n; pointId++) {
		int nextId = (pointId + 1) % n;
		Vector2D point0 = ccwPoints[pointId];
		Vector2D point1 = ccwPoints[nextId];
		int deltaWindingNumber = (point0.y < point1.y) ? 1 : -1;

		// Check if edge is fully below or above the point's horizontal level
		if (aespa_lib::genutil::signum(point0.y - point.y) == aespa_lib::genutil::signum(point1.y - point.y)) {
			continue;
		}

		// Check if edge is vertical
		if (aespa_lib::genutil::isWithin(point0.x, point1.x, epsilon)) {
			// Check if edge is to the left
			if (point0.x < point.x) continue;

			// Increment winding number
			windingNumber += deltaWindingNumber;
			continue;
		}

		// Check if edge intersects the ray to the left of the point
		if (aespa_lib::genutil::rangeMap(point.y, point0.y, point1.y, point0.x, point1.x) < point.x) {
			continue;
		}

		// Increment winding number
		windingNumber += deltaWindingNumber;
	}
	return windingNumber;
}

bool Polygon2D::containsPoint(Vector2D point) {
	return getWindingNumber(point) != 0;
}


}
}
