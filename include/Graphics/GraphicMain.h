#pragma once

#include "vex.h"
#include <iostream>
#include <vector>

using std::string, std::vector, std::pair;//, std::swap;

namespace gfxmain {
	extern double EuclideanDist(double x0, double y0, double x1, double y1);
	extern double Signum(double i);

	extern double getCrossProduct(double v1[], double v2[]);
	extern double getCrossProduct(double initPnt[], double p1[], double p2[]);
	extern double getTriangleArea(double p1[], double p2[], double p3[]);
	extern bool pointInTriangle(double pt[], double p1[], double p2[], double p3[]);

	extern color brightenColor(color col, double scale);

	extern vector<double> getLineIntersection(double startX0, double startY0, double endX0, double endY0, double startX1, double startY1, double endX1, double endY1);
	extern vector<double> getClosestPointOnLine(double pointX, double pointY, double startX, double startY, double endX, double endY);
	extern vector<double> getClosestPointOnLineSegment(double pointX, double pointY, double startX, double startY, double endX, double endY);
}
