#include "Graphics/GraphicMain.h"
#include "Graphics/GUIs/ShapesGui.h"
#include "Graphics/GUIs/SlidersGui.h"
#include <algorithm>

using namespace gfxmain;
using namespace std;

namespace {
	// Comparison for sorting values from small to big
	bool comp(pair<double, ShapeGui *> a, pair<double, ShapeGui *> b) {
		return a.first < b.first;
	}
}

// Constructor for the slider
SliderGui::SliderGui(double lowLimit, double upLimit, vector<pair<double, ShapeGui *>> defaultVal, double lowX, double lowY, double upX, double upY) {
	minVal = lowLimit;
	maxVal = upLimit;
	values = defaultVal;

	startX = lowX;
	startY = lowY;
	finishX = upX;
	finishY = upY;
}

string SliderGui::getClassName() {
	return "SliderGui";
}

// Draws the slider
void SliderGui::draw() {
	if (!isVisible()) {
		return;
	}
	// Clear line
	Brain.Screen.setPenWidth(15);
	Brain.Screen.setPenColor(black);
	double ang = atan2(finishY - startY, finishX - startX);
	Brain.Screen.drawLine(startX - 10 * cos(ang), startY - 10 * sin(ang), finishX + 10 * cos(ang), finishY + 10 * sin(ang));

	// Draw slider line
	Brain.Screen.setPenWidth(1);
	Brain.Screen.setPenColor(white);
	Brain.Screen.drawLine(startX, startY, finishX, finishY);

	// Draw sliders
	for (int i = 0; i < (int) values.size(); i++) {
		updateCord(i);
		values[i].second->draw();
	}
}

// Adds a slider button to the slider
void SliderGui::addSliderButton(double sliderVal, ShapeGui *sliderButton) {
	values.push_back(make_pair(sliderVal, sliderButton));
}

// Returns the value represented by a position on the slider
double SliderGui::getValue(double x, double y) {
	return minVal + (maxVal - minVal) * EuclideanDist(x, y, startX, startY) / EuclideanDist(startX, startY, finishX, finishY);
}
// Moves a slider to a position that represents the current value
void SliderGui::updateCord(int sliderId) {
	double currentVal = values[sliderId].first;
	double newX = startX + (finishX - startX) * (currentVal - minVal) / (maxVal - minVal);
	double newY = startY + (finishY - startY) * (currentVal - minVal) / (maxVal - minVal);
	values[sliderId].second->setCord(newX, newY);
}
// Sets and moves a slider to a position that represents the value
void SliderGui::setCord(int sliderId, double val) {
	values[sliderId].first = val;
	updateCord(sliderId);
}

void SliderGui::check() {
	if (Brain.Screen.pressing()) {
		check(Brain.Screen.xPosition(), Brain.Screen.yPosition());
	}
}

// Checks and updates sliders based on mouse position
void SliderGui::check(double mouseX, double mouseY) {
	// Check if mouse is on the slider
	vector<double> cord = getClosestPointOnLineSegment(mouseX, mouseY, startX, startY, finishX, finishY);
	if (cord.empty()) return;
	double dist = EuclideanDist(cord[0], cord[1], mouseX, mouseY);
	if (dist < 15) {
		// Find nearest slider
		double pointVal = getValue(cord[0], cord[1]);
		sort(values.begin(), values.end(), comp);
		// int sliderId = 0;
		int sliderId = lower_bound(values.begin(), values.end(), make_pair(pointVal, values.front().second)) - values.begin();
		if (sliderId == (int) values.size()) sliderId--;
		else if (sliderId == 0);
		else if (fabs(pointVal - values[sliderId - 1].first) < fabs(pointVal - values[sliderId].first)) sliderId--;

		// Modify slider's value and position
		setCord(sliderId, pointVal);

		draw();
	}
}
