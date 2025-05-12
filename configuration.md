## Robot Configuration

Modify robot configurations in [robot-config.cpp](./src/robot-config.cpp):

1. Declare motors & VEX components in [robot-config.h](./include/robot-config.h)

> [!NOTE]
>
> Devices declared in header files should have "**extern**" keyword in front of them to prevent redefinition error.<br>


2. Define those components in [robot-config.cpp](./src/robot-config.cpp), specifying the desired ports and configurations

> [!TIP]
>
> By assigning an unused port to `emptyPort`, you can use **emptyPort** and **emptyExpanderPort** to create dummy devices.


## Chassis Configuration

Modify robot specifications, chassis sensors, odometry, and auton settings in [chassis-config.cpp](./src/chassis-config.cpp).

> [!NOTE]
>
> In [chassis-config.h](./include/chassis-config.h), you'll find the global variables `botInfo`, `mainOdometry`, and `robotChassis`.<br>
> `botInfo` and `mainOdometry` don't have to be there, but they're made global for coding convenience.


### Robot Specifications

Create `botInfo` by specifying:

1. Track width in holes (distance from left wheel to right wheel, measured from center)
2. Wheel diameter in inches
3. Gear ratio from wheel to motor (e.g. wheel gear teeth / motor gear teeth)
4. Motor velocity in rpm (600 for blue cartridge, etc.)
5. Motor acceleration in rpm/s (may vary with each robot)

```cpp
BotInfo botInfo(
	23, // track width (holes)
	3.25, // wheel diameter (inches)
	48.0 / 36.0, // wheel to motor gear ratio
	600, // motor rpm
	600 * 1.5 // motor rpm/s
);
```

> [!IMPORTANT]
>
> Make sure that for substeps 1-4 configs have correct units and match the actual robot!


### Sensors & Odometry

1. Create `Encoder` sensors:

```cpp
/* (in anonymous namespace) */

// Sensors
RotationSensor look_rotationBeats(LookRotation);
RotationSensor right_rotationBeats(RightRotation);
OpticalShaftEncoder right_opticalBeats(RightEncoder);
Motor lookLeft_motorBeats(LeftMotorA);
Motor lookRight_motorBeats(RightMotorA);
```

> [!NOTE]
>
> For encoder sensors, define either `RotationSensor`, `OpticalShaftEncoder`, or `Motor` objects with the corresponding VEX devices.


2. Define a `TrackingWheel` object for each encoder sensor:
	1. The encoder sensor object
	2. Measured direction in `PolarAngle` units (right: 0°, front: 180°)

	> [!TIP]
	>
	> Use **0_polarDeg** or **180_polarDeg** for horizontal (right) tracking wheels,
	> and **90_polarDeg** or **-90_polarDeg** for vertical (look) tracking wheels.

	3. Gear ratio from encoder to wheel (e.g. sensor gear teeth / wheel gear teeth)
	4. Wheel diameter in inches
	5. Center offset in inches (see [tracking-wheel.h](./include/Aespa-Lib/Ningning-Sensors/tracking-wheel.h) for more detail)

```cpp
/* (in anonymous namespace) */

// Tracking wheels
TrackingWheel lookLeft_trackingWheel(
	lookLeft_motorBeats, 90_polarDeg, botInfo.motorToWheel_gearRatio, botInfo.wheelDiameter_inches, -botInfo.trackWidth_inches / 2
);
TrackingWheel lookRight_trackingWheel(
	lookRight_motorBeats, 90_polarDeg, botInfo.motorToWheel_gearRatio, botInfo.wheelDiameter_inches, botInfo.trackWidth_inches / 2
);
TrackingWheel look1_trackingWheel(look_rotationBeats, 90_polarDeg, 1, 2.75, -0.013);
TrackingWheel right1_trackingWheel(right_rotationBeats, 0_polarDeg, 1, 2.00, 2.0);
```

> [!IMPORTANT]
>
> **Measured direction** should be the direction of movement where the encoder sensor's reading will increase.<br>
> The sign of the **center offset** is partially determined by the measured direction.


3. Create `mainOdometry`:
	1. Add `TrackingWheel` objects
	2. Add VEX `inertial` sensors
	3. Set the position factor to `1.0 / field::tileLengthIn`<br>
	This makes `Odometry` return position in tiles, allowing [chassis move](./include/Pas1-Lib/Chassis/Move/) to function as normal

```cpp
// Odometry
Odometry mainOdometry = Odometry()
.addTrackingWheel(lookLeft_trackingWheel)
.addTrackingWheel(lookRight_trackingWheel)
.addTrackingWheel(right1_trackingWheel)
.addInertialSensor(InertialSensor, 0, 0)
.setPositionFactor(1.0 / field::tileLengthIn);
```

> [!TIP]
>
> Refer to [LemLib's Odometry configuration](https://lemlib.readthedocs.io/en/stable/tutorials/2_configuration.html#odometry) for recommendations on what sensors to use.<br>
> Currently, the `Odometry` class doesn't support heading tracking by parallel tracking wheels.


### Auton Settings

Create an `AutonSettings` object by specifying:

1. `SimpleFeedForward` Velocity feedforward (tiles/sec to volt)
2. `PIDController` Position feedback (tiles to tiles/sec)
3. `PIDController` Velocity feedback (tiles/sec error to volt)
4. `PIDController` Linear PID (tiles to velocity pct)
5. `PIDController` Angular PID (degrees to velocity pct)
6. `SlewController` Linear slew (pct/sec)
7. `SlewController` Angular slew (pct/sec)
8. `SlewController` Motor slew (pct/sec)
9. `PatienceController` Linear patience (tiles)
10. `PatienceController` Angular patience (degrees)
11. `bool` Use relative rotation for auton - any value is fine since it may be overwritten later


```cpp
/* (in anonymous namespace) */

// Auton settings
AutonSettings autonSettings(
	SimpleFeedforward(0, 12.0 / botInfo.maxVel_tilesPerSec, 0.03), // feedforward (tiles/sec to volt)
	PIDController(3.0), // position feedback (tiles to tiles/sec)
	PIDController(1.0), // velocity feedback (tiles/sec to volt)
	PIDController(100, 0, 0, 0.06, 0.05), // linear pid (tiles to pct)
	PIDController(2, PID_kI_params(0.1, 7.5, true), 0.15, 7, 0.1), // angular pid (degrees to pct)
	SlewController(1000), // linear slew (pct/sec)
	SlewController(1000), // angular slew (pct/sec)
	SlewController(1000), // motor slew (pct/sec)
	PatienceController(30, 0.001, false), // linear patience (tiles)
	PatienceController(30, 0.5, false), // angular patience (degrees)
	false // relative rotation
);
```

> [!TIP]
>
> To tune the feedforward and PID controllers, check out:<br>
> [document by 96969Y](https://docs.google.com/document/d/1gGQUS_ICVblglQYYHCu_tF6tr0dZSVkgfC9vMcKO9wQ/edit?usp=sharing)
> for feedforward tuning<br>
> [document by 2029C](https://docs.google.com/document/d/1-BjN275RGUCx_rw3oZya8FP3pveYBZwiozITEoHG33Y/edit?tab=t.0)
> for PID tuning


### Final Chassis

Finally, create the `robotChassis` using `mainOdometry`, `botInfo`, `autonSettings`, and the drivetrain motors.

```cpp
// Chassis
Differential robotChassis = Differential(
	mainOdometry, botInfo, autonSettings,
	LeftMotors, RightMotors
);
```
