using namespace vex;

extern brain Brain;

// Devices
extern controller Controller1;
extern controller Controller2;

extern motor LeftMotorA;
extern motor LeftMotorB;
extern motor LeftMotorC;
extern motor RightMotorA;
extern motor RightMotorB;
extern motor RightMotorC;
extern motor_group LeftMotors;
extern motor_group RightMotors;
extern motor_group LeftRightMotors;

extern motor IntakeMotor1;
extern motor IntakeMotor2;
extern motor_group IntakeMotors;

extern motor ArmMotor;

extern triport Expander1;

extern pneumatics FrontWingsPneumatic;
extern pneumatics LeftWingPneumatic;
extern pneumatics RightWingPneumatic;

extern pneumatics IntakeLiftPneumatic;
extern pneumatics HangPneumatic;
extern pneumatics GoalClampPneumatic;
extern pneumatics BotArmPneumatics;
extern pneumatics SwordPneumatics;
extern pneumatics Sword2Pneumatics;
extern pneumatics RedirectPneumatics;

extern encoder LookEncoder;
extern encoder RightEncoder;
extern rotation LookRotation;
extern rotation RightRotation;

extern rotation ArmRotationSensor;

extern inertial InertialSensor;
extern distance DistanceSensor;

extern distance RingDistanceSensor;
extern optical RingOpticalSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
