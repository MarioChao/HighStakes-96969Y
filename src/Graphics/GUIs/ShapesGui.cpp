#include "Graphics/GUIs/ShapesGui.h"

ShapeGui::ShapeGui(ShapeType shape, double x, double y, color fillCol, color outlineCol, double strokeWeight) {
	shapeName = shape;

	centerX = x;
	centerY = y;

	fillColor = fillCol;
	outlineColor = outlineCol;
	displayedColor = fillColor;
	outlineWidth = strokeWeight;
}

string ShapeGui::getClassName() {
	return "ShapeGui";
}

void ShapeGui::draw() {
	printf("base draw\n");
	return;
}

void ShapeGui::draw(color fillCol) {
	displayedColor = fillCol;
	this->draw();
	displayedColor = fillColor;
}

void ShapeGui::setCord(double x, double y) {
	centerX = x;
	centerY = y;
}

double ShapeGui::getX() {
	return centerX;
}

double ShapeGui::getY() {
	return centerY;
}

color ShapeGui::getColor() {
	return fillColor;
}

bool ShapeGui::isTouching(double x, double y) {
	printf("base istouching\n");
	return 0;
}
