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
const int emptyPort = PORT16;
triport EmptyExpander(emptyPort);
triport::port emptyExpanderPort(EmptyExpander.B);


/* ---------- Devices ---------- */

// Wheel motors

motor LeftMotorA(PORT1, ratio6_1, true);
motor LeftMotorB(PORT2, ratio6_1, true);
motor LeftMotorC(PORT3, ratio6_1, true);
motor RightMotorA(PORT5, ratio6_1, false);
motor RightMotorB(PORT11, ratio6_1, false);
motor RightMotorC(PORT14, ratio6_1, false);
motor_group LeftMotors(LeftMotorA, LeftMotorB, LeftMotorC);
motor_group RightMotors(RightMotorA, RightMotorB, RightMotorC);
motor_group LeftRightMotors(
	LeftMotorA, LeftMotorB, LeftMotorC,
	RightMotorA, RightMotorB, RightMotorC
);


// Other motors

motor IntakeMotor1(PORT12, ratio6_1);
motor IntakeMotor2(emptyPort, ratio18_1, true);
motor_group IntakeMotors(IntakeMotor1, IntakeMotor2);

motor ArmMotor1(PORT17, ratio18_1, false);
motor ArmMotor2(PORT15, ratio18_1, true);
motor_group ArmMotors(ArmMotor1, ArmMotor2);


// Pneumatics

pneumatics FrontWingsPneumatic(emptyExpanderPort);
pneumatics LeftWingPneumatic(emptyExpanderPort);
pneumatics RightWingPneumatic(emptyExpanderPort);

pneumatics IntakeLiftPneumatic(Brain.ThreeWirePort.G);
pneumatics HangPneumatic(emptyExpanderPort);
pneumatics GoalClampPneumatic(Brain.ThreeWirePort.H);
pneumatics BotArmPneumatics(emptyExpanderPort);
pneumatics RedirectPneumatics(emptyExpanderPort);

pneumatics LeftSword_pneumatics(Brain.ThreeWirePort.E);
pneumatics LeftSword2_pneumatics(emptyExpanderPort);
pneumatics RightSword_pneumatics(Brain.ThreeWirePort.F);
pneumatics RightSword2_pneumatics(emptyExpanderPort);

pneumatics ClimbPTO_pneumatics(emptyExpanderPort);
pneumatics ClimbHook_pneumatics(emptyExpanderPort);


// Sensors

encoder LookEncoder(emptyExpanderPort);
encoder RightEncoder(emptyExpanderPort);
rotation LookRotation(emptyPort);
rotation RightRotation(PORT10);

rotation ArmRotationSensor(PORT13, false);

inertial InertialSensor(PORT21);
distance DistanceSensor(emptyPort);

distance RingDistanceSensor(emptyPort);
optical RingOpticalSensor(PORT4);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
	// Nothing to initialize
}
