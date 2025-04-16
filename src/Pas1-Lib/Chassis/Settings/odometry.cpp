#include "Pas1-Lib/Chassis/Settings/odometry.h"

#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"


namespace {

using aespa_lib::datas::Linegular;
using aespa_lib::sensor_beats::TrackingWheel;
using aespa_lib::units::PolarAngle;
using namespace aespa_lib::units::literals::angle;
using aespa_lib::util::DriftCorrection;

const double cosAngleWithinRange = 1e-2;
const double integralSmallAngle_degrees = 8;

bool errorPrinted[3] = {};

}


namespace pas1_lib {
namespace chassis {
namespace settings {

/* ---------- Public functions ---------- */

Odometry::Odometry(
	std::vector<std::reference_wrapper<TrackingWheel>> trackingWheels,
	std::vector<std::reference_wrapper<inertial>> inertialSensors
)
	: trackedPose(0, 0, 0) {
	this->trackingWheels.clear();
	positionSensor_count = 0;

	this->inertialSensors.clear();
	inertialSensor_driftCorrections.clear();
	inertialSensor_oldMeasurements.clear();
	inertialSensor_newMeasurements.clear();
	inertialSensor_count = 0;

	for (TrackingWheel &wheel : trackingWheels) {
		addTrackingWheel(wheel);
	}
	for (inertial &sensor : inertialSensors) {
		addInertialSensor(sensor);
	}

	positionFactor = 1;
	isStarted = false;
}

Odometry::Odometry()
	: Odometry({}, {}) {}

Odometry &Odometry::addTrackingWheel(TrackingWheel &tracking_wheel) {
	// Double check if not started
	if (isStarted) {
		return *this;
	}

	// Store values
	trackingWheels.push_back(tracking_wheel);

	// Method chaining
	return *this;
}

Odometry &Odometry::addInertialSensor(inertial &sensor, double perClockwiseRevolutionDrift, double perCCWRevolutionDrift) {
	// Double check if not started
	if (isStarted) {
		return *this;
	}

	// Store values
	inertialSensors.push_back(sensor);
	inertialSensor_driftCorrections.push_back(DriftCorrection(sensor, perClockwiseRevolutionDrift, perCCWRevolutionDrift));

	// Method chaining
	return *this;
}

Odometry &Odometry::setPositionFactor(double inchToValue_ratio) {
	positionFactor = inchToValue_ratio;

	// Method chaining
	return *this;
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
	positionSensor_count = (int) trackingWheels.size();

	// Initialize old measurements
	for (int i = 0; i < positionSensor_count; i++) {
		trackingWheels[i].get().storeDistance();
	}

	/* Inertial sensors */

	// Update sensors count
	inertialSensor_count = (int) inertialSensors.size();

	// Initialize old measurements with counter-clockwise being positive, assuming right turn type
	inertialSensor_oldMeasurements.resize(inertialSensor_count);
	for (int i = 0; i < inertialSensor_count; i++) {
		inertialSensor_driftCorrections[i].setInitial();
		// inertialSensor_oldMeasurements[i] = -inertialSensors[i]->rotation(deg);
		inertialSensor_oldMeasurements[i] = -inertialSensor_driftCorrections[i].getRotation();
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
	getNewInertialSensorMeasurements();


	/* Measurement differences */

	// Get delta rotation from averages
	double deltaPolarAngle_degrees = getDeltaPolarAngle_degrees();

	// Get local delta distance from averages
	auto localDeltaRightLook = getLocalDeltaXY_inches(deltaPolarAngle_degrees);
	double localDeltaRight = localDeltaRightLook.first;
	double localDeltaLook = localDeltaRightLook.second;
	Linegular deltaDistances(localDeltaRight, localDeltaLook, deltaPolarAngle_degrees);


	/* Local to Absolute */

	bool useForwardEuler = true;
	if (useForwardEuler) {
		// Rotate by half angle (euler integration)
		// see https://docs.ftclib.org/ftclib/master/kinematics/odometry
		deltaDistances.rotateXYBy(deltaPolarAngle_degrees / 2);
	} else {
		// Rotate with pose exponential
		deltaDistances.rotateExponentialBy(deltaPolarAngle_degrees);
	}


	// Rotate to absolute difference
	PolarAngle rightPolarAngle = trackedPose.getRotation() - 90_polarDeg;
	deltaDistances.rotateXYBy(rightPolarAngle);


	/* Update */

	// Update old sensor values
	inertialSensor_oldMeasurements = inertialSensor_newMeasurements;

	// Update odometry values
	Linegular deltaPose(deltaDistances.getX(), deltaDistances.getY(), deltaPolarAngle_degrees);
	trackedPose += deltaPose;
}

void Odometry::setPosition_raw(double x, double y) {
	trackedPose.setPosition(x, y);
}

void Odometry::setPosition_scaled(double x, double y) {
	setPosition_raw(x / positionFactor, y / positionFactor);
}

void Odometry::setLookAngle(aespa_lib::units::PolarAngle polarAngle) {
	trackedPose.setRotation(polarAngle);
}

void Odometry::setLookAngle_field(double fieldAngles_degrees) {
	trackedPose.setRotation(aespa_lib::angle::swapFieldPolar_degrees(fieldAngles_degrees));
}

void Odometry::setLookPose_raw(Linegular pose) {
	setPosition_raw(pose.getX(), pose.getY());
	setLookAngle(pose.getRotation());
}

void Odometry::setLookPose_scaled(Linegular pose) {
	setLookPose_raw(pose / positionFactor);
}

double Odometry::getX_scaled() { return trackedPose.getX() * positionFactor; }

double Odometry::getY_scaled() { return trackedPose.getY() * positionFactor; }

PolarAngle Odometry::getLookRotation() { return trackedPose.getRotation(); }

Linegular Odometry::getLookPose_scaled() {
	return Linegular(getX_scaled(), getY_scaled(), getLookRotation());
}

void Odometry::printDebug() {
	// Print tracked values
	printf("Track X: %07.3f, Y: %07.3f, Ang: %07.3f\n", getX_scaled(), getY_scaled(), getLookRotation().polarDeg());

	// Print position sensor readings
	for (int i = 0; i < positionSensor_count; i++) {
		double m = trackingWheels[i].get().getDistance_inches();
		printf("POS %2d: %07.3f\n", i, m);
	}

	// Print inertial sensor readings
	for (int i = 0; i < inertialSensor_count; i++) {
		double m = inertialSensor_newMeasurements[i];
		printf("INR %2d: %07.3f\n", i, m);
	}
}


/* ---------- Private functions ---------- */

void Odometry::odometryThread() {}

void Odometry::getNewInertialSensorMeasurements() {
	/* Inertial sensors */
	inertialSensor_newMeasurements.resize(inertialSensor_count);
	for (int i = 0; i < inertialSensor_count; i++) {
		inertialSensor_driftCorrections[i].correct();
		inertialSensor_newMeasurements[i] = -inertialSensor_driftCorrections[i].getRotation();
	}
}

double Odometry::getDeltaPolarAngle_degrees() {
	double totalDeltaAngle = 0;
	for (int i = 0; i < inertialSensor_count; i++) {
		// Angle difference
		double deltaAngle_degrees = inertialSensor_newMeasurements[i] - inertialSensor_oldMeasurements[i];

		// Add to total
		totalDeltaAngle += deltaAngle_degrees;
	}

	// Return
	if (inertialSensor_count == 0) {
		if (!errorPrinted[2]) {
			printf("Error: no inertial sensors available.\n");
			errorPrinted[2] = 1;
		}
		return 0;
	}
	return totalDeltaAngle / inertialSensor_count;
}

std::pair<double, double> Odometry::getLocalDeltaXY_inches(double deltaPolarAngle_degrees) {
	double totalDeltaX_inches = 0;
	double totalDeltaY_inches = 0;
	int xValidSensorsCount = 0;
	int yValidSensorsCount = 0;
	for (int i = 0; i < positionSensor_count; i++) {
		// Get tracking wheel data
		TrackingWheel &trackingWheel = trackingWheels[i].get();
		double wheelDeltaDistance_inches = trackingWheel.getRawDeltaDistance_inches(true);
		PolarAngle wheelDirection = trackingWheel.getDirection();

		// Get tracking center delta distance
		double centerDeltaDistance_inches = trackingWheel.getCenterDeltaDistance_inches(wheelDeltaDistance_inches, deltaPolarAngle_degrees);
		// printf("wheelDelta: %.3f, centerDelta: %.3f, dAng: %.3f\n", wheelDeltaDistance_inches, centerDeltaDistance_inches, deltaPolarAngle_degrees);

		// Variables
		double cosAngle;
		bool isMeetCondition;

		// ----- Delta X -----
		// equation: localDeltaX * cos(angle) = centerDeltaTranslate
		// thus: localDeltaX = centerDeltaTranslate / cos(angle)
		// condition: cos(angle) ≠ 0

		// Check condition
		cosAngle = cos(wheelDirection.polarRad());
		isMeetCondition = !aespa_lib::genutil::isWithin(cosAngle, 0, cosAngleWithinRange);
		if (isMeetCondition) {
			// Add to total
			double localDeltaX = centerDeltaDistance_inches / cosAngle;
			totalDeltaX_inches += localDeltaX;
			xValidSensorsCount++;
		}

		// ----- Delta Y -----
		// equation: localDeltaY * cos(90 - angle) = centerDeltaTranslate
		// thus: localDeltaY = centerDeltaTranslate / cos(90 - angle)
		// condition: cos(90 - angle) ≠ 0
		// note: cos(a) = cos(-a)

		// Check condition
		cosAngle = cos(aespa_lib::genutil::toRadians(90 - wheelDirection.polarDeg()));
		isMeetCondition = !aespa_lib::genutil::isWithin(cosAngle, 0, cosAngleWithinRange);
		if (isMeetCondition) {
			// Add to total
			double localDeltaY = centerDeltaDistance_inches / cosAngle;
			totalDeltaY_inches += localDeltaY;
			yValidSensorsCount++;
		}
	}

	// Return
	if (xValidSensorsCount == 0) {
		if (!errorPrinted[0]) {
			printf("Error: no position sensors available for delta X.\n");
			errorPrinted[0] = 1;
		}
		xValidSensorsCount = 1;
	}
	if (yValidSensorsCount == 0) {
		if (!errorPrinted[1]) {
			printf("Error: no position sensors available for delta Y.\n");
			errorPrinted[1] = 1;
		}
		yValidSensorsCount = 1;
	}
	return { totalDeltaX_inches / xValidSensorsCount, totalDeltaY_inches / yValidSensorsCount };
}

}
}
}
