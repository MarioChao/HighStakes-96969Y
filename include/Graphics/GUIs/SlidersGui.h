#pragma once

#include "GuiClass.h"

class ShapeGui;

class SliderGui : public GuiClass {
protected:
	virtual void draw() override;
private:
	// Attributes
	double minVal, maxVal;
	vector<pair<double, ShapeGui *>> values;

	double startX, startY;
	double finishX, finishY;
public:
	// Constructor
	SliderGui(double lowLimit, double upLimit, vector<pair<double, ShapeGui *>> defaultVal, double lowX, double lowY, double upX, double upY);

	// Function
	void addSliderButton(double sliderVal, ShapeGui *sliderButton);

	double getValue(double x, double y);
	void updateCord(int sliderId);
	void setCord(int sliderId, double val);

	virtual void check() override;
	void check(double mouseX, double mouseY);
};
