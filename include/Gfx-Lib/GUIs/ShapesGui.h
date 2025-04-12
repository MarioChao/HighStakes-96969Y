#pragma once

#include "GuiClass.h"

namespace shape {
	enum ShapeType {
		RECTANGLE,
		ROUNDED_RECTANGLE,
		TRIANGLE, // Currently doesn't support fill
		ROUNDED_TRIANGLE, // Same as Triangle (currently)
		CIRCLE
	};
}

using namespace shape;

class ShapeGui : public GuiClass {
public:
	// Constructor
	ShapeGui(ShapeType shape, double x, double y, color fillCol, color outlineCol, double strokeWeight);

	// Function
	virtual string getClassName() override;

	virtual void draw() override;
	void draw(color fillCol);

	void setCord(double x, double y);
	double getX();
	double getY();

	color getColor();

	virtual bool isTouching(double x, double y);

protected:
	// Inherited attributes
	double centerX, centerY;

	color fillColor, outlineColor;
	color displayedColor;
	double outlineWidth;

private:
	// Attributes
	ShapeType shapeName;
};

class Rectangle : public ShapeGui {
protected:
	// Attributes
	double width, height;
public:
	// Constructor
	Rectangle(double x, double y, double width, double height, color fillCol, color outlineCol, double outlineWeight);

	// Function
	virtual void draw() override;
	virtual bool isTouching(double x, double y) override;
};

class RoundedRectangle : public ShapeGui {
protected:
	// Attributes
	double width, height;
	double cornerRadius;
public:
	// Constructor
	RoundedRectangle(double x, double y, double width, double height, double radius, color fillCol, color outlineCol, double outlineWeight);

	// Function
	virtual void draw() override;
	virtual bool isTouching(double x, double y) override;
};

class Triangle : public ShapeGui {
protected:
	// Attributes
	double point1[2], point2[2], point3[2]; // Basically doesn't use the inherited centerX and centerY
public:
	// Constructor
	Triangle(double x, double y, double p1[], double p2[], double p3[], color fillCol, color outlineCol, double outlineWeight);

	// Function
	virtual void draw() override;
	virtual bool isTouching(double x, double y) override;
};

class RoundedTriangle : public ShapeGui {
protected:
	// Attributes
	double point1[2], point2[2], point3[2];
	double cornerRadius;
public:
	// Constructor
	RoundedTriangle(double x, double y, double naught[], double supported[], double yet[], double radius, color fillCol, color outlineCol, double outlineWeight);

	// Function
	virtual void draw() override;
	virtual bool isTouching(double x, double y) override;
};

class Circle : public ShapeGui {
protected:
	// Attributes
	double radius;
public:
	// Constructor
	Circle(double x, double y, double radius, color fillCol, color outlineCol, double outlineWeight);

	// Function
	virtual void draw() override;
	virtual bool isTouching(double x, double y) override;
};
