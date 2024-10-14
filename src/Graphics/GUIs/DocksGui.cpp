#include "Graphics/GUIs/DocksGui.h"

DockGui::DockGui(double leftX, double topY, double width, double height, vector<GuiClass *> guiList, vector<void (*)()> functionList) {
	this->leftX = leftX;
	this->topY = topY;
	this->width = width;
	this->height = height;
	guis = guiList;
	functions = functionList;
}

void DockGui::addGui(GuiClass *gui) {
	guis.push_back(gui);
}

void DockGui::addGuis(vector<GuiClass *> guiList) {
	for (GuiClass *gui : guiList) {
		addGui(gui);
	}
}

/// @brief Add funtions that run when dock is enabled.
/// @param func The callback function to be ran when setEnabled(true) is called.
void DockGui::addEnabledFunction(void (*func)()) {
	onEnabledFunctions.push_back(func);
}

/// @brief Add functions that run when dock is enabled and check() is called.
/// @param func The callback function to be ran when dock is enabled and check() is called.
void DockGui::addFunction(void (*func)()) {
	functions.push_back(func);
}

/// @brief Draw every gui stored in the dock.
void DockGui::draw() {
	// Check if visible & usable
	if (!isVisible() || !enabled) {
		return;
	}
	// Draw guis
	for (GuiClass *gui : guis) {
		gui->draw();
	}
}

/// @brief Clear the dock by drawing a black rectangle with white borders.
void DockGui::clearDock() {
	// Clear dock
	Brain.Screen.setPenColor(white);
	Brain.Screen.setPenWidth(2);
	Brain.Screen.setFillColor(black);
	Brain.Screen.drawRectangle(leftX, topY, width, height);
}

/// @brief Check if the dock is enabled, then run the dock functions.
void DockGui::check() {
	if (enabled) {
		// Run functions
		for (void (*func)() : functions) {
			func();
		}
		// Check guis
		for (GuiClass *gui : guis) {
			gui->check();
		}
	}
}

/// @brief Set the dock as enabled (active) or not (inactive).
/// @param enabled Whether the dock is active or not.
void DockGui::setEnabled(bool enabled) {
	this->enabled = enabled;
	if (enabled) {
		// printf("Enabled dock\n");
		// On dock enabled
		clearDock();
		// Set gui visibility & draw
		for (GuiClass *gui : guis) {
			gui->setVisibility(true);
		}
		draw();
		// Enabled functions
		for (void (*func)() : onEnabledFunctions) {
			func();
		}
	} else {
		// printf("Disabled dock\n");
		// On dock disabled
		for (GuiClass *gui : guis) {
			gui->setVisibility(false);
		}
	}
}

/// @brief Get the enabled (active/inactive) state of the dock.
/// @return Whether the dock is active or not.
bool DockGui::getEnabled() {
	return enabled;
}
