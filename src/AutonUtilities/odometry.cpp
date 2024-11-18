#include "AutonUtilities/odometry.h"
#include "AutonUtilities/driftCorrection.h"
#include "Utilities/robotInfo.h"
#include "Utilities/fieldInfo.h"
#include "Utilities/generalUtility.h"
#include "main.h"

// File-local variables

namespace {
	const double cosAngleWithinRange = 1e-5;
}


// Public functions

Odometry::Odometry() {
	positionSensor_polarAngles_degrees.clear();
	positionSensor_RevolutionCallbacks.clear();
	positionSensor_sensorToWheel_gearRatios.clear();
	positionSensor_wheelDiameters_inches.clear();
	positionSensor_normalRotateRadii_inches.clear();
	positionSensor_oldMeasurements.clear();
	positionSensor_newMeasurements.clear();
	positionSensor_count = 0;

	inertialSensors.clear();
	inertialSensor_driftCorrections.clear();
	inertialSensor_oldMeasurements.clear();
	inertialSensor_newMeasurements.clear();
	inertialSensor_count = 0;

	positionFactor = 1;
	isStarted = false;

	x = y = 0;
	fieldAngle_degrees = 0;
}

void Odometry::addPositionSensor2D(double polarAngle, double (*revolutionCallback)(), double sensorToWheel_gearRatio, double wheelDiameter_inches, double normalRotateRadius_inches) {
	// Double check if not started
	if (isStarted) {
		return;
	}

	// Store values
	positionSensor_polarAngles_degrees.push_back(polarAngle);
	positionSensor_RevolutionCallbacks.push_back(revolutionCallback);
	positionSensor_sensorToWheel_gearRatios.push_back(sensorToWheel_gearRatio);
	positionSensor_wheelDiameters_inches.push_back(wheelDiameter_inches);
	positionSensor_normalRotateRadii_inches.push_back(normalRotateRadius_inches);
}

void Odometry::addInertialSensor(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift) {
	// Double check if not started
	if (isStarted) {
		return;
	}

	// Store values
	inertialSensors.push_back(&sensor);
	inertialSensor_driftCorrections.push_back(new DriftCorrection(sensor, perClockwiseRevolutionDrift, perCCWRevolutionDrift));
}

void Odometry::setPositionFactor(double inchToValue_ratio) {
	positionFactor = inchToValue_ratio;
}

void Odometry::startThreads() {
	// task odometryTask(odometryThread);
	// NOTE: task doesn't accept lamdas with captures, and workarounds are quite complicated
}

void Odometry::start() {
	// Double check if not started
	if (isStarted) {
		return;
	}

	// Set started state
	isStarted = true;

	/* Position sensors */

	// Update sensors count
	positionSensor_count = (int) positionSensor_polarAngles_degrees.size();

	// Initialize old measurements
	positionSensor_oldMeasurements.resize(positionSensor_count);
	for (int i = 0; i < positionSensor_count; i++) {
		positionSensor_oldMeasurements[i] = positionSensor_RevolutionCallbacks[i]();
	}

	/* Inertial sensors */

	// Update sensors count
	inertialSensor_count = (int) inertialSensors.size();

	// Initialize old measurements with counter-clockwise being positive, assuming right turn type
	inertialSensor_oldMeasurements.resize(inertialSensor_count);
	for (int i = 0; i < inertialSensor_count; i++) {
		inertialSensor_oldMeasurements[i] = -inertialSensors[i]->rotation(deg);
		inertialSensor_driftCorrections[i]->setInitial();
	}
}

void Odometry::odometryFrame() {
	// Make sure started
	if (!isStarted) {
		start();
	}

	/* Sensor values */

	// Get new sensor values
	getNewPositionSensorMeasurements();
	getNewInertialSensorMeasurements();


	/* Measurement differences */

	// Get rotation difference from averages
	double deltaPolarAngle_degrees = getDeltaPolarAngle_degrees();

	// Get local distance difference from averages, multiplied by position factor
	double localDeltaRight = getLocalDeltaX_inches(deltaPolarAngle_degrees) * positionFactor;
	double localDeltaLook = getLocalDeltaY_inches(deltaPolarAngle_degrees) * positionFactor;


	/* Local to Absolute */

	// Rotate to absolute difference
	double averageAngleDegrees = getPolarAngle_degrees() + deltaPolarAngle_degrees / 2;
	double localToGlobalRotateAngle = genutil::toRadians(averageAngleDegrees);
	double absoluteDeltaRight = localDeltaRight * cos(localToGlobalRotateAngle) - localDeltaLook * sin(localToGlobalRotateAngle);
	double absoluteDeltaLook = localDeltaRight * sin(localToGlobalRotateAngle) + localDeltaLook * cos(localToGlobalRotateAngle);
	printf("test: %.3f %.3f %.3f\n", genutil::toDegrees(localToGlobalRotateAngle), absoluteDeltaRight, absoluteDeltaLook);


	/* Update */

	// Update old sensor values
	positionSensor_oldMeasurements = positionSensor_newMeasurements;
	inertialSensor_oldMeasurements = inertialSensor_newMeasurements;

	// Update odometry values
	x += absoluteDeltaRight;
	y += absoluteDeltaLook;
	fieldAngle_degrees -= deltaPolarAngle_degrees;
}

void Odometry::setValues(double x, double y, double fieldAngles) {
	this->x = x;
	this->y = y;
	this->fieldAngle_degrees = fieldAngles;
}

double Odometry::getX() { return x; }

double Odometry::getY() { return y; }

double Odometry::getFieldAngle_degrees() { return fieldAngle_degrees; }

double Odometry::getPolarAngle_degrees() {
	return 90 - fieldAngle_degrees;
}


// Private functions

void Odometry::odometryThread() {}

void Odometry::getNewPositionSensorMeasurements() {
	/* Position sensors */
	positionSensor_newMeasurements.resize(positionSensor_count);
	for (int i = 0; i < positionSensor_count; i++) {
		positionSensor_newMeasurements[i] = positionSensor_RevolutionCallbacks[i]();
	}
}

void Odometry::getNewInertialSensorMeasurements() {
	/* Inertial sensors */
	inertialSensor_newMeasurements.resize(inertialSensor_count);
	for (int i = 0; i < inertialSensor_count; i++) {
		inertialSensor_driftCorrections[i]->correct();
		inertialSensor_newMeasurements[i] = -inertialSensors[i]->rotation(deg);
		inertialSensor_driftCorrections[i]->setInitial();
	}
}

double Odometry::getDeltaPolarAngle_degrees() {
	double totalDeltaAngle = 0;
	for (int i = 0; i < inertialSensor_count; i++) {
		totalDeltaAngle += inertialSensor_newMeasurements[i] - inertialSensor_oldMeasurements[i];
	}
	return totalDeltaAngle / inertialSensor_count;
}

double Odometry::getLocalDeltaX_inches(double deltaPolarAngle_degrees) {
	double totalDeltaX_inches = 0;
	int validSensorsCount = 0;
	for (int i = 0; i < positionSensor_count; i++) {
		// equation: localDeltaX * cos(angle) = sensorDeltaTranslate
		// thus: localDeltaX = sensorDeltaTranslate / cos(angle)
		// condition: cos(angle) ≠ 0

		// Check condition
		double cosAngle = cos(genutil::toRadians(positionSensor_polarAngles_degrees[i]));
		if (genutil::isWithin(cosAngle, 0, cosAngleWithinRange)) {
			continue;
		}

		// Calculate measured raw difference in inches
		double measuredDeltaDistance = positionSensor_newMeasurements[i] - positionSensor_oldMeasurements[i]; // sensor revolutions
		measuredDeltaDistance *= positionSensor_sensorToWheel_gearRatios[i]; // wheel revolutions
		measuredDeltaDistance *= M_PI * positionSensor_wheelDiameters_inches[i]; // wheel travel distance

		// Decrease by rotated arc distance
		double rotatedDeltaDistance = positionSensor_normalRotateRadii_inches[i] * genutil::toRadians(deltaPolarAngle_degrees);
		double sensorDeltaTranslate = measuredDeltaDistance - rotatedDeltaDistance;

		// Add to total
		double localDeltaX = sensorDeltaTranslate / cosAngle;
		totalDeltaX_inches += localDeltaX;
		validSensorsCount++;
	}
	return totalDeltaX_inches / validSensorsCount;
}

double Odometry::getLocalDeltaY_inches(double deltaAngle_degrees) {
	double totalDeltaY_inches = 0;
	int validSensorsCount = 0;
	for (int i = 0; i < positionSensor_count; i++) {
		// equation: localDeltaY * cos(90 - angle) = sensorDeltaTranslate
		// thus: localDeltaY = sensorDeltaTranslate / cos(90 - angle)
		// condition: cos(90 - angle) ≠ 0

		// Check condition
		double cosAngle = cos(genutil::toRadians(90 - positionSensor_polarAngles_degrees[i]));
		if (genutil::isWithin(cosAngle, 0, cosAngleWithinRange)) {
			continue;
		}

		// Calculate measured raw difference in inches
		double measuredDeltaDistance = positionSensor_newMeasurements[i] - positionSensor_oldMeasurements[i]; // sensor revolutions
		measuredDeltaDistance *= positionSensor_sensorToWheel_gearRatios[i]; // wheel revolutions
		measuredDeltaDistance *= M_PI * positionSensor_wheelDiameters_inches[i]; // wheel travel distance

		// Decrease by rotated arc distance
		double rotatedDeltaDistance = positionSensor_normalRotateRadii_inches[i] * genutil::toRadians(deltaAngle_degrees);
		double sensorDeltaTranslate = measuredDeltaDistance - rotatedDeltaDistance;

		// Add to total
		double localDeltaY = sensorDeltaTranslate / cosAngle;
		totalDeltaY_inches += localDeltaY;
		validSensorsCount++;
	}
	return totalDeltaY_inches / validSensorsCount;
}
