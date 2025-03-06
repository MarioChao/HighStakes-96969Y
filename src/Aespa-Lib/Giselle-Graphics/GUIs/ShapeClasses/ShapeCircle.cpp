#include "Graphics/GUIs/ShapesGui.h"
#include "Graphics/GraphicMain.h"

Circle::Circle(double x, double y, double radius, color fillCol, color outlineCol, double outlineWeight)
	: ShapeGui(ShapeType::CIRCLE, x, y, fillCol, outlineCol, outlineWeight) {
	this->radius = radius;
}

void Circle::draw() {
	if (!isVisible()) {
		return;
	}
	Brain.Screen.setPenColor(outlineColor);
	Brain.Screen.setPenWidth(outlineWidth);
	Brain.Screen.setFillColor(displayedColor);
	Brain.Screen.drawCircle(centerX, centerY, radius);
}

bool Circle::isTouching(double x, double y) {
	return gfxmain::EuclideanDist(centerX, centerY, x, y) <= radius;
}
