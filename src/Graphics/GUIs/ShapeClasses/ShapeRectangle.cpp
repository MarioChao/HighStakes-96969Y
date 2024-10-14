#include "Graphics/GUIs/ShapesGui.h"

Rectangle::Rectangle(double x, double y, double width, double height, color fillCol, color outlineCol, double outlineWeight)
	: ShapeGui(ShapeType::RECTANGLE, x, y, fillCol, outlineCol, outlineWeight) {
	this->width = width;
	this->height = height;
}

void Rectangle::draw() {
	if (!isVisible()) {
		return;
	}

	double halfW = width / 2;
	double halfH = height / 2;

	// Draw sides
	// Brain.Screen.setPenWidth(0);
	Brain.Screen.setPenWidth(outlineWidth);
	Brain.Screen.setPenColor(outlineColor);
	Brain.Screen.setFillColor(displayedColor);
	Brain.Screen.drawRectangle(centerX - halfW, centerY - halfH, width, height);
}

bool Rectangle::isTouching(double x, double y) {
	double halfW = width / 2;
	double halfH = height / 2;

	if (x < centerX - halfW || x > centerX + halfW) return false;
	if (y < centerY - halfH || y > centerY + halfH) return false;

	return true;
}
