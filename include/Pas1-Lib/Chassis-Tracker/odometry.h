#pragma once

#include "main.h"
#include "Pas1-Lib/Chassis-Tracker/tracking-wheel.h"
#include "AutonUtilities/driftCorrection.h"

class DriftCorrection;
class Linegular;

namespace chassis_tracker {
	class Odometry;
}

class chassis_tracker::Odometry {
public:
	Odometry(
		std::vector<std::reference_wrapper<TrackingWheel>> trackingWheels,
		std::vector<std::reference_wrapper<inertial>> inertialSensors = {}
	);
	Odometry();

	/**
	 * @brief Adds a tracking wheel for the chassis.
	 * 
	 * Only works before the odometry is started.
	 * 
	 * @param tracking_wheel The tracking wheel object.
	 */
	Odometry &addTrackingWheel(TrackingWheel &tracking_wheel);

	/**
	 * @brief Adds an inertial sensor to track the chassis's rotation in a 2D plane.
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
	Odometry &addInertialSensor(inertial &sensor, double perClockwiseRevolutionDrift = 0, double perCCWRevolutionDrift = 0);

	/**
	 * @brief Sets the factor multiplied to the robot's position in inches.
	 * 
	 * @param inchToValue_ratio The conversion ratio (target value) / (inches). For example, (1 tile) / (23.5625 inches).
	 */
	Odometry &setPositionFactor(double inchToValue_ratio);

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
	// Tracking wheels
	std::vector<std::reference_wrapper<TrackingWheel>> trackingWheels;

	int positionSensor_count;

	// Inertial gyro sensors
	std::vector<std::reference_wrapper<inertial>> inertialSensors;
	std::vector<DriftCorrection> inertialSensor_driftCorrections;

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

	void getNewInertialSensorMeasurements();

	double getDeltaPolarAngle_degrees();
	std::pair<double, double> getLocalDeltaXY_inches(double deltaPolarAngle_degrees);
};
