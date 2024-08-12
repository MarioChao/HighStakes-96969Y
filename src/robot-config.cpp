#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// Devices

controller Controller1(primary);
controller Controller2(partner);

// Wheel motors

motor LeftMotorA(PORT6, ratio6_1, true);
motor LeftMotorB(PORT4, ratio6_1, true);
motor LeftMotorC(PORT5, ratio6_1);
motor RightMotorA(PORT3, ratio6_1);
motor RightMotorB(PORT1, ratio6_1);
motor RightMotorC(PORT2, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA, RightMotorB, RightMotorC);

// Intake motors

motor IntakeMotor1(PORT18, ratio6_1);
motor IntakeMotor2(PORT19, ratio6_1);
motor_group IntakeMotors(IntakeMotor1, IntakeMotor2);

// Expander

triport Expander1(PORT22);

// Wing pneumatic (not used)

pneumatics FrontWingsPneumatic(Expander1.A);
pneumatics LeftWingPneumatic(Expander1.B);
pneumatics RightWingPneumatic(Expander1.C);

pneumatics IntakeLiftPneumatic(Brain.ThreeWirePort.A);

pneumatics GoalClampPneumatic(Brain.ThreeWirePort.C);

// Sensors

encoder LookEncoder(Expander1.G); // .G .H
rotation LookRotation(PORT9);
rotation RightRotation(PORT10);

inertial InertialSensor(PORT8);
distance DistanceSensor(PORT20);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
