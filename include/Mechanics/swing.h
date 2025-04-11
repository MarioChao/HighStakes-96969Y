#pragma once

namespace swing {
void runThread();

void preauton();

void setState_left(int, double = 0);
void set2ndState_left(int, double = 0);
void setState_right(int, double = 0);
void set2ndState_right(int, double = 0);

void switchState_left();
void switch2ndState_left();
void switchState_right();
void switch2ndState_right();

void control(int);

bool canControl();

extern int _taskState;
extern double _taskDelay;
}
