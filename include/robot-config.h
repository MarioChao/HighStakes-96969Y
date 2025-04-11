#pragma once


// Brain & Controller

extern vex::brain Brain;

extern vex::controller Controller1;
extern vex::controller Controller2;


// Dummy

extern const int emptyPort;
extern vex::triport EmptyExpander;
extern vex::triport::port emptyExpanderPort;


// Devices

extern vex::motor LeftMotorA;
extern vex::motor LeftMotorB;
extern vex::motor LeftMotorC;
extern vex::motor RightMotorA;
extern vex::motor RightMotorB;
extern vex::motor RightMotorC;
extern vex::motor_group LeftMotors;
extern vex::motor_group RightMotors;
extern vex::motor_group LeftRightMotors;

extern vex::motor IntakeMotor1;
extern vex::motor IntakeMotor2;
extern vex::motor_group IntakeMotors;

extern vex::motor ArmMotor1;
extern vex::motor ArmMotor2;
extern vex::motor_group ArmMotors;

extern vex::pneumatics FrontWingsPneumatic;
extern vex::pneumatics LeftWingPneumatic;
extern vex::pneumatics RightWingPneumatic;

extern vex::pneumatics IntakeLiftPneumatic;
extern vex::pneumatics HangPneumatic;
extern vex::pneumatics GoalClampPneumatic;
extern vex::pneumatics BotArmPneumatics;
extern vex::pneumatics RedirectPneumatics;

extern vex::pneumatics LeftSword_pneumatics;
extern vex::pneumatics LeftSword2_pneumatics;
extern vex::pneumatics RightSword_pneumatics;
extern vex::pneumatics RightSword2_pneumatics;

extern vex::pneumatics ClimbPTO_pneumatics;
extern vex::pneumatics ClimbHook_pneumatics;

extern vex::encoder LookEncoder;
extern vex::encoder RightEncoder;
extern vex::rotation LookRotation;
extern vex::rotation RightRotation;

extern vex::rotation ArmRotationSensor;

extern vex::inertial InertialSensor;
extern vex::distance DistanceSensor;

extern vex::distance RingDistanceSensor;
extern vex::optical RingOpticalSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
