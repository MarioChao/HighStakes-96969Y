#include "Graphics/GUIs/GuiClass.h"

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
