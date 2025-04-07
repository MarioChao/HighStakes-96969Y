#include "vex.h"


namespace {
using namespace vex;
}


/* ---------- Brain & Controller ---------- */

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller1(primary);
controller Controller2(partner);


/* ---------- Dummy ---------- */

// PORT22 is used for the Brain's default ThreeWirePort
const int emptyPort = PORT20;
triport EmptyExpander(emptyPort);


/* ---------- Devices ---------- */

// Wheel motors

motor LeftMotorA(PORT8, ratio6_1, true);
motor LeftMotorB(PORT9, ratio6_1, true);
motor LeftMotorC(PORT10, ratio6_1, true);
motor RightMotorA(PORT1, ratio6_1, false);
motor RightMotorB(PORT2, ratio6_1, false);
motor RightMotorC(PORT3, ratio6_1, false);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(
	LeftMotorA, LeftMotorB, LeftMotorC,
	RightMotorA, RightMotorB, RightMotorC
);


// Other motors

motor IntakeMotor1(PORT4, ratio6_1);
motor IntakeMotor2(emptyPort, ratio18_1, true);
motor_group IntakeMotors(IntakeMotor1, IntakeMotor2);

motor ArmMotor(emptyPort, ratio36_1);


// Pneumatics

pneumatics FrontWingsPneumatic(EmptyExpander.A);
pneumatics LeftWingPneumatic(EmptyExpander.B);
pneumatics RightWingPneumatic(EmptyExpander.C);

pneumatics IntakeLiftPneumatic(EmptyExpander.B);
pneumatics HangPneumatic(EmptyExpander.E);
pneumatics GoalClampPneumatic(Brain.ThreeWirePort.F);
pneumatics BotArmPneumatics(EmptyExpander.B);
pneumatics SwordPneumatics(EmptyExpander.B);
pneumatics Sword2Pneumatics(EmptyExpander.B);
pneumatics RedirectPneumatics(EmptyExpander.B);

pneumatics ClimbPTO_pneumatics(EmptyExpander.B);
pneumatics ClimbHook_pneumatics(EmptyExpander.B);


// Sensors

encoder LookEncoder(EmptyExpander.A);
encoder RightEncoder(EmptyExpander.A);
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
