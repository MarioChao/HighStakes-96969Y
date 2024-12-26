#include "Graphics/GUIs/DocksGui.h"

DockGui::DockGui(double leftX, double topY, double width, double height, vector<GuiClass *> guiList, vector<void (*)()> functionList) {
	this->leftX = leftX;
	this->topY = topY;
	this->width = width;
	this->height = height;
	guis = guiList;
	functions = functionList;
}

string DockGui::getClassName() {
	return "DockGui";
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
		// printf(":(, %d, %d\n", !isVisible(), !enabled);
		return;
	}
	// printf("DRAW S %d\n", (int) guis.size());

	// Draw guis
	for (GuiClass *gui : guis) {
		gui->draw();
	}
}

/// @brief Clear the dock by drawing a black rectangle with white borders.
void DockGui::clearDock() {
	// Clear dock
	Brain.Screen.setPenColor(white);
	Brain.Screen.setPenWidth(1);
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
		// Clear screen
		clearDock();

		// Show dock
		this->setVisibility(true);

		// Show guis
		for (GuiClass *gui : guis) {
			gui->setVisibility(true);
			if (gui->getClassName() == "DockGui") {
				((DockGui *) gui)->setEnabled(true);
			}
		}
		draw();

		// Enabled functions
		for (void (*func)() : onEnabledFunctions) {
			func();
		}
	} else {
		// Hide guis
		this->setVisibility(false);
		for (GuiClass *gui : guis) {
			gui->setVisibility(false);
			if (gui->getClassName() == "DockGui") {
				((DockGui *) gui)->setEnabled(false);
			}
		}
	}
}

/// @brief Get the enabled (active/inactive) state of the dock.
/// @return Whether the dock is active or not.
bool DockGui::getEnabled() {
	return enabled;
}
