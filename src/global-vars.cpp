#include "global-vars.h"

#include "Utilities/fieldInfo.h"


namespace {
using pas1_lib::planning::trajectories::TrajectoryPlanner;
using pas1_lib::planning::profiles::SplineProfile;
using aespa_lib::datas::NamedStorage;
}


// ---------- Global variables ----------

competition Competition;

int intakePartType = 1;

bool isArmPneumatic = false;

timer drivingTimer;


/* Autonomous */

RobotSimulator robotSimulator;
bool mainUseSimulator = false;

TrajectoryPlanner testTrajectoryPlan;
timer trajectoryTestTimer;

// Spline Profile Storage
NamedStorage<SplineProfile> splineProfile_storage;
