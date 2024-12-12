#pragma once

#include "main.h"

class DriftCorrection;
class Linegular;

class Odometry {
public:
	/**
	 * @brief Construct a new Odometry object.
	 * 
	 */
	Odometry();

	/**
	 * @brief Adds a sensor to track the robot's position in a 2D plane.
	 * 
	 * Example: `(90, []() {return sensor.position(rev);}, 1, 2, 0)` for a look/forward direction sensor with 1:1 gear ratio, 2" diameter, and 0" normal rotate radius.
	 * 
	 * Only works before the odometry is started.
	 * 
	 * @param polarAngle The sensor's measuring direction, in degrees, on the 2D plane, with x-axis = 0 and counter-clockwise being positive.
	 * @param revolutionCallback A function pointer for getting the sensor's immediate value in revolutions.
	 * @param sensorToWheel_gearRatio The ratio `sensorGearTeeth` / `wheelGearTeeth`, or (sensor gear) / (farther gear) * (close gear) / (far gear) * ... * () / ().
	 * @param wheelDiameter_inches The diameter of the driven wheel in inches.
	 * @param normalRotateRadius_inches The distance between the sensor's measuring line and a parallel line passing through the tracking center. Positive means the sensor is measuring forward to the right of the tracking center.
	 */
	void addPositionSensor2D(double polarAngle, double (*revolutionCallback)(), double sensorToWheel_gearRatio, double wheelDiameter_inches, double normalRotateRadius_inches);

	/**
	 * @brief Adds an inertial sensor to track the robot's rotation in a 2D plane.
	 * The inertial sensor should have a "right" turnType.
	 * 
	 * It's recommended to only use 1 inertial sensor with drift correction factors.
	 * 
	 * Only works before the odometry is started.
	 * 
	 * @param sensor The inertial sensor used to track robot's rotation.
	 * @param perClockwiseRevolutionDrift The drift, in degrees, per clockwise revolution of the robot.
	 * @param perCCWRevolutionDrift The drift, in degrees, per counter-clockwise revolution of the robot.
	 */
	void addInertialSensor(inertial &sensor, double perClockwiseRevolutionDrift = 0, double perCCWRevolutionDrift = 0);

	/**
	 * @brief Sets the factor multiplied to the robot's position in inches.
	 * 
	 * @param inchToValue_ratio The conversion ratio (target value) / (inches). For example, (1 tile) / (23.5625 inches).
	 */
	void setPositionFactor(double inchToValue_ratio);

	/**
	 * @brief (unavailable) Starts tracking the robot's position. This can only be called once.
	 * 
	 * For now, please use `odometryFrame()`.
	 * 
	 */
	void startThreads();

	/// @brief Starts tracking the robot's position.
	void start();

	/// @brief Restarts tracking the robot's position.
	void restart();

	/// @brief Calculate robot's new position from the change in sensor measurements since the last call.
	void odometryFrame();

	/**
	 * @brief Sets the robot's position in a 2D plane.
	 * 
	 * @param x Horizontal position with position factor.
	 * @param y Vertical position with position factor.
	 */
	void setPosition(double x, double y);

	/**
	 * @brief Sets the robot's rotation (look direction) in a 2D plane.
	 * 
	 * @param fieldAngles_degrees Robot's front/look direction. Field angle rotation in degrees.
	 */
	void setLookAngle(double fieldAngles_degrees);

	/**
	 * @brief Sets the robot's rotation (look direction) in a 2D plane.
	 * 
	 * @param fieldAngles_degrees Robot's right direction. Field angle rotation in degrees.
	 */
	void setRightAngle(double fieldAngle_degrees);

	double getX();
	double getY();
	double getLookFieldAngle_degrees();
	double getRightFieldAngle_degrees();

	Linegular getLookLinegular();

	void printDebug();

private:
	// Position sensors
	std::vector<double> positionSensor_polarAngles_degrees;
	std::vector<double (*)()> positionSensor_RevolutionCallbacks;
	std::vector<double> positionSensor_sensorToWheel_gearRatios;
	std::vector<double> positionSensor_wheelDiameters_inches;
	std::vector<double> positionSensor_normalRotateRadii_inches;

	std::vector<double> positionSensor_oldMeasurements, positionSensor_newMeasurements;

	int positionSensor_count;

	// Inertial gyro sensors
	std::vector<inertial *> inertialSensors;
	std::vector<DriftCorrection *> inertialSensor_driftCorrections;

	std::vector<double> inertialSensor_oldMeasurements, inertialSensor_newMeasurements;

	int inertialSensor_count;

	// Factors
	double positionFactor;

	// Starting state
	bool isStarted = false;

	// Tracked values
	double x, y;
	double right_fieldAngle_degrees;

	// Functions
	void odometryThread();

	void getNewPositionSensorMeasurements();
	void getNewInertialSensorMeasurements();

	double getDeltaPolarAngle_degrees();
	double getLocalDeltaX_inches(double deltaPolarAngle_degrees);
	double getLocalDeltaY_inches(double deltaPolarAngle_degrees);
};
