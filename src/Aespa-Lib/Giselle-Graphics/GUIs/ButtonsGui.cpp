// Buttons Gui Class

#include "Graphics/GraphicMain.h"
#include "Graphics/GUIs/ShapesGui.h"
#include "Graphics/GUIs/ButtonsGui.h"

using gfxmain::brightenColor;

// Constructor
ButtonGui::ButtonGui(ShapeGui *shape, string msg, color textFill, void (*func)()) {
	buttonShape = shape;

	centerX = buttonShape->getX();
	centerY = buttonShape->getY();

	fillColor = buttonShape->getColor();
	disabledColor = brightenColor(fillColor, 0.3);
	unusableColor = brightenColor(fillColor, 0.5);
	unusableDisabledColor = brightenColor(fillColor, 0.1);
	displayedColor = fillColor;

	storedText = displayedText = msg;
	textColor = textFill;

	runFunction = func;
}

ButtonGui::ButtonGui(double x, double y, double width, double height, double radius, color fillCol, color outlineCol, double strokeWeight, string msg, color textFill, void (*func)()) {
	centerX = x;
	centerY = y;
	/*
	outlineColor = outlineCol;
	outlineWidth = strokeWeight;*/

	buttonShape = new RoundedRectangle(x, y, width, height, radius, fillCol, outlineCol, strokeWeight);

	fillColor = fillCol;
	disabledColor = brightenColor(fillCol, 0.3);
	unusableColor = brightenColor(fillCol, 0.5);
	unusableDisabledColor = brightenColor(fillCol, 0.1);
	displayedColor = fillColor;

	storedText = displayedText = msg;
	textColor = textFill;

	runFunction = func;

	dbFrames = 0;
}
ButtonGui::ButtonGui() {
	ButtonGui(0, 0, 20, 5, 1, color(100, 100, 100), color::black, 2, "Button", color::black, [] { return; });
}

string ButtonGui::getClassName() {
	return "ButtonGui";
}

/// @brief Draw the button shape & text.
void ButtonGui::draw() {
	if (!isVisible()) {
		return;
	}
	buttonShape->draw(displayedColor);

	// Draw text
	Brain.Screen.setPenColor(textColor);
	Brain.Screen.setFont(monoS);
	Brain.Screen.printAt(centerX - (int) displayedText.length() * 4, centerY, 0, displayedText.c_str());
}

void ButtonGui::check() {
	dbFrames++;
	dbFrames = fmin(dbFrames, 20 + 1);
	if (Brain.Screen.pressing()) {
		// printf("Pressed: %3ld, %3ld\n", Brain.Screen.xPosition(), Brain.Screen.yPosition());
		check(Brain.Screen.xPosition(), Brain.Screen.yPosition());
	}
}

/// @brief Check for button click, with debounce.
/// @param clickX The x-position of the cursor.
/// @param clickY The y-position of the cursor.
void ButtonGui::check(double clickX, double clickY) {
	if (!buttonShape->isTouching(clickX, clickY)) return;
	if (dbFrames < 20 || !usable) {
		dbFrames = 0;
		return;
	}
	dbFrames = 0;
	runFunction();
}

/// @brief Set the state of the button as "enabled" (different from "usable").
void ButtonGui::enable() {
	if (!enabled) {
		enabled = true;
		displayedText = storedText;
		displayedColor = fillColor;
		draw();
	}
}
/// @brief Set the state of the button as "disabled" (different from "unusable").
void ButtonGui::disable() {
	if (enabled) {
		enabled = false;
		displayedColor = disabledColor;
		draw();
	}
}
/// @brief Get the enabled state of the button (different from "usability").
/// @return The current enabled state of the button.
bool ButtonGui::getEnabled() {
	return enabled;
}

/// @brief Set whether the button can be clicked or not (different from "enabled").
/// @param usability Whether the button is functional or not.
void ButtonGui::setUsability(bool usability) {
	usable = usability;
	if (usable == false) {
		if (enabled) {
			displayedColor = unusableColor;
		} else {
			displayedColor = unusableDisabledColor;
		}
	} else {
		if (enabled) {
			displayedColor = fillColor;
		} else {
			displayedColor = disabledColor;
		}
	}
	draw();
}
/// @brief Get the usability of the button.
/// @return Whether the button is functional or not.
bool ButtonGui::getUsability() {
	return usable;
}

void ButtonGui::activateButtonFunction() {
	runFunction();
}

string ButtonGui::getDisplayedText() {
	return displayedText;
}
