#pragma once

#include "GuiClass.h"

class DockGui : public GuiClass {
private:
	// Attributes
	double leftX, topY;
	double width, height;

	vector<GuiClass *> guis;
	vector<void (*)()> onEnabledFunctions;
	vector<void (*)()> functions;

	bool enabled = true;
public:
	// Constructor
	DockGui(double leftX, double topY, double width, double height, vector<GuiClass *> guiList, vector<void (*)()> functionList);

	// Functions
	void addGui(GuiClass *gui);
	void addGuis(vector<GuiClass *> guiList);
	void addEnabledFunction(void (*func)());
	void addFunction(void (*func)());

	virtual void draw() override;
	void clearDock();

	virtual void check() override;

	void setEnabled(bool enabled);
	bool getEnabled();
};
