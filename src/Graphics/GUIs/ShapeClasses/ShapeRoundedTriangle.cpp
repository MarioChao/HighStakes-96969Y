#include "Graphics/GUIs/ShapesGui.h"
#include "Graphics/GraphicMain.h"

RoundedTriangle::RoundedTriangle(double x, double y, double p1[], double p2[], double p3[], double radius, color fillCol, color outlineCol, double outlineWeight)
	: ShapeGui(ShapeType::ROUNDED_TRIANGLE, x, y, fillCol, outlineCol, outlineWeight) {
	copy(p1, p1 + 2, point1);
	copy(p2, p2 + 2, point2);
	copy(p3, p3 + 2, point3);
	cornerRadius = radius;
}

void RoundedTriangle::draw() {
	if (!isVisible()) {
		return;
	}

	// Draw border
	Brain.Screen.setPenWidth(outlineWidth);
	Brain.Screen.setPenColor(outlineColor);
	Brain.Screen.drawLine(point1[0], point1[1], point2[0], point2[1]);
	Brain.Screen.drawLine(point2[0], point2[1], point3[0], point3[1]);
	Brain.Screen.drawLine(point3[0], point3[1], point1[0], point1[1]);
}

bool RoundedTriangle::isTouching(double x, double y) {
	double tmpP[] = { x, y };
	return gfxmain::pointInTriangle(tmpP, point1, point2, point3);
}
