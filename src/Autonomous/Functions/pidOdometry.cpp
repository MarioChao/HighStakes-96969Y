#include "Autonomous/autonFunctions.h"

#include "Pas1-Lib/Auton/Control-Loops/pid.h"
#include "Pas1-Lib/Auton/End-Conditions/patience.h"
#include "Pas1-Lib/Auton/End-Conditions/timeout.h"
#include "Pas1-Lib/Chassis/Move/global-move-to.h"

#include "Mechanics/botDrive.h"

#include "Aespa-Lib/Karina-Data-Structures/linegular.h"
#include "Aespa-Lib/Winter-Utilities/angle.h"
#include "Aespa-Lib/Winter-Utilities/general.h"

#include "Utilities/fieldInfo.h"

#include "global-vars.h"

namespace {

using aespa_lib::datas::Linegular;
using namespace pas1_lib::chassis::move;

}


namespace autonfunctions {

void runLinearPIDPath(std::vector<std::vector<double>> waypoints, double maxVelocity, bool isReverse) {
	Linegular lg(0, 0, 0);
	for (std::vector<double> point : waypoints) {
		// Rotation
		global::turnToFace(robotChassis, global::turnToFace_params(point[0], point[1], isReverse), false);

		// Linear
		lg = mainOdometry.getLookPose_scaled();
		global::driveToPoint(robotChassis, global::driveToPoint_params(point[0], point[1], isReverse, maxVelocity), false);

		// Info
		lg = mainOdometry.getLookPose_scaled();
		printf("ED: X: %.3f, Y: %.3f\n", lg.getX(), lg.getY());
	}
}

}
