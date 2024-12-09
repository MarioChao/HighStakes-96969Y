#pragma once

#include "GuiClass.h"

class ShapeGui;

class ButtonGui : public GuiClass {
public:
	// Constructor
	ButtonGui(ShapeGui *shape, string msg, color textFill, void (*func)());
	ButtonGui(double x, double y, double width, double height, double radius, color fillCol, color outlineCol, double strokeWeight, string msg, color textFill, void (*func)());
	ButtonGui();

	virtual string getClassName() override;

	// Draw
	virtual void draw() override;

	// Check
	virtual void check() override;
	void check(double clickX, double clickY);

	// Enable / disable
	void enable();
	void disable();
	bool getEnabled();

	// Usability
	void setUsability(bool usability);
	bool getUsability();

	// Button function
	void activateButtonFunction();

	// Access
	string getDisplayedText();

protected:
private:
	// Attributes
	double centerX, centerY;

	double outlineWidth;
	color outlineColor;
	color fillColor;

	ShapeGui *buttonShape;
	color disabledColor, unusableColor, unusableDisabledColor;
	color displayedColor;

	string storedText, displayedText;
	color textColor;

	void (*runFunction)();

	bool enabled = true; // On/off state
	bool usable = true; // Can be tapped

	int dbFrames;
};
