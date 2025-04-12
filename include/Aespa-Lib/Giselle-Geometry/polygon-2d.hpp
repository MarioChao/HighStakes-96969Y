#pragma once

#include <vector>
#include "Aespa-Lib/Giselle-Geometry/vector-2d.hpp"


namespace aespa_lib {
namespace geometry {


struct Polygon2D {
public:
	Polygon2D(std::vector<Vector2D> ccwPoints);

	double getArea();
	int getWindingNumber(Vector2D point);
	bool containsPoint(Vector2D point);


	std::vector<Vector2D> ccwPoints;
};


}
}
