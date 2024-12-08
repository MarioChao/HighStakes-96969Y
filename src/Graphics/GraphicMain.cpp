#include "Utilities/fieldInfo.h"
#include "Graphics/GraphicMain.h"

namespace gfxmain {
	// Functions
	// Returns the euclidean distance between point (x0, y0) and (x1, y1)
	double EuclideanDist(double x0, double y0, double x1, double y1) {
		return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
	}

	// Returns the unit sign of the number
	double Signum(double i) {
		return (i > 0) - (i < 0);
	}

	// Returns the cross product of two vectors
	double getCrossProduct(double v1[], double v2[]) {
		// cross-product = (v1.x * v2.y) - (v1.y * v2.x)
		return (v1[0] * v2[1]) - (v1[1] * v2[0]);
	}

	// Returns the cross product given an origin and two coordinates
	double getCrossProduct(double initPnt[], double p1[], double p2[]) {
		double v1[] = { p1[0] - initPnt[0], p1[1] - initPnt[1] };
		double v2[] = { p2[0] - initPnt[0], p2[1] - initPnt[1] };
		return getCrossProduct(v1, v2);
	}

	// Returns the area of a triangle given the coordinates of the vertices
	double getTriangleArea(double p1[], double p2[], double p3[]) {
		// Translate p1 to (0, 0) ==> cross-product of p1-p2 and p1-p3
		// Area = cross-product / 2
		return getCrossProduct(p1, p2, p3) / 2.0;
	}

	// Returns whether the given point is inside a triangle defined by three vertices
	bool pointInTriangle(double pt[], double p1[], double p2[], double p3[]) {
		// Reference from https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
		double sign1, sign2, sign3;
		sign1 = getCrossProduct(pt, p1, p2);
		sign2 = getCrossProduct(pt, p2, p3);
		sign3 = getCrossProduct(pt, p3, p1);
		bool hasNeg = (sign1 < 0) || (sign2 < 0) || (sign3 < 0);
		bool hasPos = (sign1 > 0) || (sign2 > 0) || (sign3 > 0);
		return !(hasNeg && hasPos);
	}

	// Returns a color with rgb scaled by a certain factor
	color brightenColor(color col, double scale) {
		double r = col.rgb() / (1 << 16);
		double g = col.rgb() % (1 << 16) / (1 << 8);
		double b = col.rgb() % (1 << 8);
		r *= scale;
		g *= scale;
		b *= scale;
		r = fmax(fmin(r, 255), 0);
		g = fmax(fmin(g, 255), 0);
		b = fmax(fmin(b, 255), 0);
		return color(r, g, b);
	}

	// Calculates and returns the intersection point of two lines
	vector<double> getLineIntersection(double startX0, double startY0, double endX0, double endY0, double startX1, double startY1, double endX1, double endY1) {
		vector<double> ret = {};
		double retX, retY;
		if (startX0 == endX0) {
			if (startX1 == endX1) {
				// No or infinite intersections
			} else {
				double pathM1 = (endY1 - startY1) / (endX1 - startX1);
				double pathB1 = startY1 - pathM1 * startX1;
				retX = startX0;
				retY = pathM1 * retX + pathB1;
				ret = { retX, retY };
			}
		} else if (startX1 == endX1) {
			double pathM0 = (endY0 - startY0) / (endX0 - startX0);
			double pathB0 = startY0 - pathM0 * startX0;
			retX = startX1;
			retY = pathM0 * retX + pathB0;
			ret = { retX, retY };
		} else {
			double pathM0 = (endY0 - startY0) / (endX0 - startX0);
			double pathB0 = startY0 - pathM0 * startX0;
			double pathM1 = (endY1 - startY1) / (endX1 - startX1);
			double pathB1 = startY1 - pathM1 * startX1;
			if (pathM0 == pathM1) {
				// No or infinite intersections
			} else {
				// 1. y = mx + b; 2. y = m1x + b1
				// mx + b = m1x + b1
				// (m - m1)x = b1 - b
				// x = (b1 - b) / (m - m1)
				retX = (pathB1 - pathB0) / (pathM0 - pathM1);
				retY = pathM0 * retX + pathB0;
				ret = { retX, retY };
			}
		}
		return ret;
	}

	// Calculates and returns the closest point on a line from a point
	vector<double> getClosestPointOnLine(double pointX, double pointY, double startX, double startY, double endX, double endY) {
		vector<double> ret = {};
		if (startX == endX) {
			// Get perpendicular line through point
			double perpendM = 0;
			double perpendB = pointY;
			// Calculate intersection of two lines
			ret = getLineIntersection(0, perpendB, 1, perpendM + perpendB, startX, startY, endX, endY);
		} else if (startY == endY) {
			// Perpendicular line is vertical
			// Calculate intersection of two lines
			ret = getLineIntersection(pointX, 0, pointX, 1, startX, startY, endX, endY);
		} else {
			// Get perpendicular line through point
			double pathM = (endY - startY) / (endX - startX);
			double perpendM = -1 / pathM;
			double perpendB = pointY - perpendM * pointX;
			// Calculate intersection of two lines
			ret = getLineIntersection(0, perpendB, 1, perpendM + perpendB, startX, startY, endX, endY);
		}
		return ret;
	}

	// Calculates and returns the closest point on a line segment from a point
	vector<double> getClosestPointOnLineSegment(double pointX, double pointY, double startX, double startY, double endX, double endY) {
		vector<double> ret = getClosestPointOnLine(pointX, pointY, startX, startY, endX, endY);
		if (endX < startX) {
			std::swap(startX, endX);
			std::swap(startY, endY);
		}
		if (!ret.empty()) {
			if (ret[0] > endX) {
				ret = { endX, endY };
			} else if (ret[0] < startX) {
				ret = { startX, startY };
			}
		}
		return ret;
	}
}
