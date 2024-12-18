#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// PORT22 is used for the Brain's default ThreeWirePort

const int emptyPort = PORT4;

// Devices

controller Controller1(primary);
controller Controller2(partner);

// Wheel motors

motor LeftMotorA(PORT15, ratio6_1, true);
motor LeftMotorB(PORT17, ratio6_1, true);
motor LeftMotorC(PORT16, ratio6_1);
motor RightMotorA(PORT20, ratio6_1);
motor RightMotorB(PORT18, ratio6_1);
motor RightMotorC(PORT19, ratio6_1, true);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA,
							RightMotorB, RightMotorC);

// Intake motors

motor IntakeMotor1(PORT1, ratio6_1);
motor IntakeMotor2(PORT4, ratio18_1, true);
motor_group IntakeMotors(IntakeMotor1, IntakeMotor2);

// arm motor

motor ArmMotor(PORT10, ratio36_1);

// Expander

triport Expander1(emptyPort);

// Wing pneumatic (not used)

pneumatics FrontWingsPneumatic(Expander1.A);
pneumatics LeftWingPneumatic(Expander1.B);
pneumatics RightWingPneumatic(Expander1.C);

pneumatics IntakeLiftPneumatic(Expander1.B);
pneumatics HangPneumatic(Expander1.E);
pneumatics GoalClampPneumatic(Brain.ThreeWirePort.C);
pneumatics BotArmPneumatics(Expander1.B);
pneumatics SwordPneumatics(Brain.ThreeWirePort.G);
pneumatics Sword2Pneumatics(Brain.ThreeWirePort.H);
pneumatics RedirectPneumatics(Brain.ThreeWirePort.D);

// Sensors

encoder LookEncoder(Expander1.A);
encoder RightEncoder(Brain.ThreeWirePort.A);
rotation LookRotation(PORT14);
rotation RightRotation(emptyPort);

rotation ArmRotationSensor(PORT9, true);

inertial InertialSensor(PORT13);
distance DistanceSensor(emptyPort);

distance RingDistanceSensor(emptyPort);
optical RingOpticalSensor(PORT7);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
