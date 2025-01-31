#include "AutonUtilities/odometry.h"

#include "AutonUtilities/driftCorrection.h"
#include "AutonUtilities/linegular.h"
#include "Utilities/angleUtility.h"
#include "Utilities/fieldInfo.h"
#include "Utilities/generalUtility.h"
#include "main.h"

// File-local variables

namespace {
	const double cosAngleWithinRange = 1e-2;
	const double integralSmallAngle_degrees = 8;
	const double inertialNoiseFilter_degrees = 1e-4;
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
	right_fieldAngle_degrees = 0;
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
		inertialSensor_driftCorrections[i]->setInitial();
		// inertialSensor_oldMeasurements[i] = -inertialSensors[i]->rotation(deg);
		inertialSensor_oldMeasurements[i] = -inertialSensor_driftCorrections[i]->getRotation();
	}
}

void Odometry::restart() {
	isStarted = false;
	start();
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
	Linegular deltaDistances(localDeltaRight, localDeltaLook, deltaPolarAngle_degrees);


	/* Local to Absolute */

	if (genutil::isWithin(deltaPolarAngle_degrees, 0, integralSmallAngle_degrees)) {
		// Rotate by half angle (euler integration)
		// see https://docs.ftclib.org/ftclib/master/kinematics/odometry
		deltaDistances.rotateXYBy(genutil::toRadians(deltaPolarAngle_degrees / 2));
	} else {
		// Rotate with pose exponential
		deltaDistances.rotateExponentialBy(genutil::toRadians(deltaPolarAngle_degrees));
	}


	// Rotate to absolute difference
	double rightPolarAngle_degrees = angle::swapFieldPolar_degrees(getRightFieldAngle_degrees());
	double localToGlobalRotateAngle = genutil::toRadians(rightPolarAngle_degrees);
	deltaDistances.rotateXYBy(localToGlobalRotateAngle);


	/* Update */

	// Update old sensor values
	positionSensor_oldMeasurements = positionSensor_newMeasurements;
	inertialSensor_oldMeasurements = inertialSensor_newMeasurements;

	// Update odometry values
	x += deltaDistances.getX();
	y += deltaDistances.getY();
	right_fieldAngle_degrees -= deltaPolarAngle_degrees;
}

void Odometry::setPosition(double x, double y) {
	this->x = x;
	this->y = y;
}

void Odometry::setLookAngle(double fieldAngles_degrees) {
	this->right_fieldAngle_degrees = fieldAngles_degrees + 90.0;
}

void Odometry::setRightAngle(double fieldAngles_degrees) {
	this->right_fieldAngle_degrees = fieldAngles_degrees;
}

double Odometry::getX() { return x; }

double Odometry::getY() { return y; }

double Odometry::getLookFieldAngle_degrees() {
	return right_fieldAngle_degrees - 90.0;
}

double Odometry::getRightFieldAngle_degrees() {
	return right_fieldAngle_degrees;
}

Linegular Odometry::getLookLinegular() {
	return Linegular(x, y, angle::swapFieldPolar_degrees(getLookFieldAngle_degrees()));
}

void Odometry::printDebug() {
	// Print tracked values
	printf("Track X: %07.3f, Y: %07.3f, Ang: %07.3f\n", getX(), getY(), getLookFieldAngle_degrees());

	// Print position sensor readings
	for (int i = 0; i < positionSensor_count; i++) {
		double m = positionSensor_newMeasurements[i];
		printf("POS %2d: %07.3f\n", i, m);
	}

	// Print inertial sensor readings
	for (int i = 0; i < inertialSensor_count; i++) {
		double m = inertialSensor_newMeasurements[i];
		printf("INR %2d: %07.3f\n", i, m);
	}
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
		inertialSensor_newMeasurements[i] = -inertialSensor_driftCorrections[i]->getRotation();
	}
}

double Odometry::getDeltaPolarAngle_degrees() {
	double totalDeltaAngle = 0;
	for (int i = 0; i < inertialSensor_count; i++) {
		// Angle difference
		double deltaAngle_degrees = inertialSensor_newMeasurements[i] - inertialSensor_oldMeasurements[i];

		// Small noise filter
		// if (genutil::isWithin(deltaAngle_degrees, 0, inertialNoiseFilter_degrees)) continue;

		// Add to total
		totalDeltaAngle += deltaAngle_degrees;
	}

	// Return
	if (inertialSensor_count == 0) {
		printf("Error: no inertial sensors available.");
		return 0;
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

	// Return
	if (validSensorsCount == 0) {
		printf("Error: no position sensors available for delta X.");
		return 0;
	}
	return totalDeltaX_inches / validSensorsCount;
}

double Odometry::getLocalDeltaY_inches(double deltaPolarAngle_degrees) {
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
		double rotatedDeltaDistance = positionSensor_normalRotateRadii_inches[i] * genutil::toRadians(deltaPolarAngle_degrees);
		double sensorDeltaTranslate = measuredDeltaDistance - rotatedDeltaDistance;

		// Add to total
		double localDeltaY = sensorDeltaTranslate / cosAngle;
		totalDeltaY_inches += localDeltaY;
		validSensorsCount++;
	}

	// Return
	if (validSensorsCount == 0) {
		printf("Error: no position sensors available for delta Y.");
		return 0;
	}
	return totalDeltaY_inches / validSensorsCount;
}
