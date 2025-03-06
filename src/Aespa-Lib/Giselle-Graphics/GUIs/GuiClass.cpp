#include "Graphics/GUIs/GuiClass.h"

string GuiClass::getClassName() {
	return "GuiClass";
}

void GuiClass::draw() {
	printf("GuiClass draw\n");
	return;
}

void GuiClass::check() {
	return;
}

void GuiClass::setVisibility(bool visibility) {
	visible = visibility;
}

bool GuiClass::isVisible() {
	return visible;
}
