#pragma once

#include "vex.h"
#include <iostream>
#include <vector>

using std::string, std::vector, std::pair, std::make_pair, std::copy;

class GuiClass {
public:
	virtual string getClassName();

	virtual void draw();
	virtual void check();

	void setVisibility(bool visibility);
	bool isVisible();

private:
	bool visible = true;

	string className;
};
