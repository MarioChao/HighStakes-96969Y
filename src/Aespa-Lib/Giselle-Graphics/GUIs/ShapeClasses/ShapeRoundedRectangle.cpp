#include "Graphics/GUIs/ShapesGui.h"

RoundedRectangle::RoundedRectangle(double x, double y, double width, double height, double radius, color fillCol, color outlineCol, double outlineWeight)
	: ShapeGui(ShapeType::ROUNDED_RECTANGLE, x, y, fillCol, outlineCol, outlineWeight) {
	this->width = width;
	this->height = height;
	cornerRadius = radius;
}

void RoundedRectangle::draw() {
	if (!isVisible()) {
		return;
	}

	// Draw corner
	double halfW = width / 2;
	double halfH = height / 2;

	Brain.Screen.setPenWidth(outlineWidth);
	Brain.Screen.setPenColor(outlineColor);
	Brain.Screen.setFillColor(displayedColor);
	for (int i = -1; i <= 1; i += 2) {
		for (int j = -1; j <= 1; j += 2) {
			Brain.Screen.drawCircle(centerX + (halfW - cornerRadius) * i, centerY + (halfH - cornerRadius) * j, cornerRadius);
		}
	}

	// Draw sides
	Brain.Screen.setPenWidth(0);
	Brain.Screen.setFillColor(displayedColor);
	Brain.Screen.drawRectangle(centerX - (halfW - cornerRadius), centerY - halfH, width - 2 * cornerRadius, height);
	Brain.Screen.drawRectangle(centerX - halfW, centerY - (halfH - cornerRadius), width, height - 2 * cornerRadius);

	Brain.Screen.setPenWidth(outlineWidth);
	Brain.Screen.setPenColor(outlineColor);
	Brain.Screen.drawLine(centerX - (halfW - cornerRadius), centerY - halfH + outlineWidth / 2, centerX + (halfW - cornerRadius), centerY - halfH + outlineWidth / 2);
	Brain.Screen.drawLine(centerX - (halfW - cornerRadius), centerY + halfH + outlineWidth / 2, centerX + (halfW - cornerRadius), centerY + halfH + outlineWidth / 2);
	Brain.Screen.drawLine(centerX - halfW, centerY - (halfH - cornerRadius), centerX - halfW, centerY + (halfH - cornerRadius));
	Brain.Screen.drawLine(centerX + halfW, centerY - (halfH - cornerRadius), centerX + halfW, centerY + (halfH - cornerRadius));
}

bool RoundedRectangle::isTouching(double x, double y) {
	double halfW = width / 2;
	double halfH = height / 2;

	if (x < centerX - halfW || x > centerX + halfW) return false;
	if (y < centerY - halfH || y > centerY + halfH) return false;

	return true;
}
