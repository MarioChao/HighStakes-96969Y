#include "Pas1-Lib/Chassis-Tracker/odometry.h"

#include "Aespa-Lib/Winter-Utilities/angleUtility.h"
#include "Aespa-Lib/Winter-Utilities/generalUtility.h"

#include "AutonUtilities/driftCorrection.h"
#include "AutonUtilities/linegular.h"
#include "Utilities/fieldInfo.h"
#include "main.h"

// File-local variables

namespace {
	const double cosAngleWithinRange = 1e-2;
	const double integralSmallAngle_degrees = 8;

	bool errorPrinted[3] = {};
}

namespace chassis_tracker {
	// Public functions

	Odometry::Odometry(
		std::vector<std::reference_wrapper<TrackingWheel>> trackingWheels,
		std::vector<std::reference_wrapper<inertial>> inertialSensors
	) {
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

		x = y = 0;
		right_fieldAngle_degrees = 0;
	}

	Odometry::Odometry()
	: Odometry({}, {})
	{}

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

		// Get local delta distance from averages, multiplied by position factor
		auto localDeltaRightLook = getLocalDeltaXY_inches(deltaPolarAngle_degrees);
		double localDeltaRight = localDeltaRightLook.first;
		double localDeltaLook = localDeltaRightLook.second;
		localDeltaRight *= positionFactor;
		localDeltaLook *= positionFactor;
		Linegular deltaDistances(localDeltaRight, localDeltaLook, deltaPolarAngle_degrees);


		/* Local to Absolute */

		if (aespa_lib::genutil::isWithin(deltaPolarAngle_degrees, 0, integralSmallAngle_degrees)) {
			// Rotate by half angle (euler integration)
			// see https://docs.ftclib.org/ftclib/master/kinematics/odometry
			deltaDistances.rotateXYBy(aespa_lib::genutil::toRadians(deltaPolarAngle_degrees / 2));
		} else {
			// Rotate with pose exponential
			deltaDistances.rotateExponentialBy(aespa_lib::genutil::toRadians(deltaPolarAngle_degrees));
		}


		// Rotate to absolute difference
		double rightPolarAngle_degrees = aespa_lib::angle::swapFieldPolar_degrees(getRightFieldAngle_degrees());
		double localToGlobalRotateAngle = aespa_lib::genutil::toRadians(rightPolarAngle_degrees);
		deltaDistances.rotateXYBy(localToGlobalRotateAngle);


		/* Update */

		// Update old sensor values
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
		return Linegular(x, y, aespa_lib::angle::swapFieldPolar_degrees(getLookFieldAngle_degrees()));
	}

	void Odometry::printDebug() {
		// Print tracked values
		printf("Track X: %07.3f, Y: %07.3f, Ang: %07.3f\n", getX(), getY(), getLookFieldAngle_degrees());

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


	// Private functions

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
			double wheelDirection_polarDegrees = trackingWheel.getDirection_polarDegrees();

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
			cosAngle = cos(aespa_lib::genutil::toRadians(wheelDirection_polarDegrees));
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
			cosAngle = cos(aespa_lib::genutil::toRadians(90 - wheelDirection_polarDegrees));
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
};
