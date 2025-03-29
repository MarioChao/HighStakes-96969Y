#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

// PORT22 is used for the Brain's default ThreeWirePort

const int emptyPort = PORT20;

// Devices

controller Controller1(primary);
controller Controller2(partner);

// Wheel motors

motor LeftMotorA(PORT8, ratio6_1, true);
motor LeftMotorB(PORT9, ratio6_1, true);
motor LeftMotorC(PORT10, ratio6_1, true);
motor RightMotorA(PORT1, ratio6_1, false);
motor RightMotorB(PORT2, ratio6_1, false);
motor RightMotorC(PORT3, ratio6_1, false);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(LeftMotorA, LeftMotorB, LeftMotorC, RightMotorA,
							RightMotorB, RightMotorC);

// Intake motors

motor IntakeMotor1(PORT4, ratio6_1);
motor IntakeMotor2(emptyPort, ratio18_1, true);
motor_group IntakeMotors(IntakeMotor1, IntakeMotor2);

// arm motor

motor ArmMotor(emptyPort, ratio36_1);

// Expander

triport Expander1(emptyPort);

// Wing pneumatic (not used)

pneumatics FrontWingsPneumatic(Expander1.A);
pneumatics LeftWingPneumatic(Expander1.B);
pneumatics RightWingPneumatic(Expander1.C);

pneumatics IntakeLiftPneumatic(Expander1.B);
pneumatics HangPneumatic(Expander1.E);
pneumatics GoalClampPneumatic(Brain.ThreeWirePort.F);
pneumatics BotArmPneumatics(Expander1.B);
pneumatics SwordPneumatics(Expander1.B);
pneumatics Sword2Pneumatics(Expander1.B);
pneumatics RedirectPneumatics(Expander1.B);

pneumatics ClimbPTO_pneumatics(Brain.ThreeWirePort.H);
pneumatics ClimbHook_pneumatics(Brain.ThreeWirePort.G);

// Sensors

encoder LookEncoder(Expander1.A);
encoder RightEncoder(Expander1.A);
rotation LookRotation(emptyPort);
rotation RightRotation(PORT21);

rotation ArmRotationSensor(emptyPort, true);

inertial InertialSensor(PORT7);
distance DistanceSensor(emptyPort);

distance RingDistanceSensor(emptyPort);
optical RingOpticalSensor(emptyPort);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
