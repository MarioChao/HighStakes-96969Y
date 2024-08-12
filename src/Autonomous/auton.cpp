#include "Autonomous/auton.h"
#include "Autonomous/autonFunctions.h"
#include "Utilities/robotInfo.h"
#include "Utilities/debugFunctions.h"
#include "preauton.h"
#include "main.h"

namespace {
    using namespace auton;
    using namespace botinfo;
    using debug::printOnController;

    void autonTest();

    void runAutonRedDown();
    void runAutonRedUp();
    void runAutonBlueDown();
    void runAutonBlueUp();
    void runAutonSkills();
    void runAutonSkillsCrossBarrier();
    void runAutonSkillsStrategicPush();

    bool userRunningAutonomous = false;
    autonomousType auton_runType = autonomousType::Test;
    int auton_allianceId;
}

void setAutonRunType(int allianceId, autonomousType autonType) {
    switch (autonType) {
        case autonomousType::RedDown:
            printOnController("Auton: RedDown");
            printf("Nearaw\n");
            break;
        case autonomousType::RedUp:
            printOnController("Auton: RedUp");
            printf("Nearaw Safe\n");
            break;
        case autonomousType::BlueDown:
            printOnController("Auton: BlueDown");
            printf("Nearel\n");
            break;
        case autonomousType::BlueUp:
            printOnController("Auton: BlueUp");
            printf("Faraw\n");
            break;
        case autonomousType::AutonSkills:
            printOnController("Auton: Skills");
            printf("AuSk\n");
            break;
        case autonomousType::DrivingSkills:
            printOnController("Driving Skills");
            printf("DrSk\n");
            break;
        default:
            printOnController("Auton: None");
            printf("None\n");
            break;
    }
    auton_runType = autonType;
    auton_allianceId = allianceId;
}

void showAutonRunType() {
    setAutonRunType(auton_allianceId, auton_runType);
}

autonomousType getAutonRunType() {
    return auton_runType;
}

bool isUserRunningAuton() {
    return userRunningAutonomous;
}

void runAutonomous() {
    printf("Auton time!\n");
    userRunningAutonomous = false;
    switch (auton_runType) {
        case autonomousType::RedUp:
            runAutonRedDown();
            break;
        case autonomousType::RedDown:
            runAutonRedUp();
            break;
        case autonomousType::BlueUp:
            runAutonBlueDown();
            break;
        case autonomousType::BlueDown:
            runAutonBlueUp();
            break;
        case autonomousType::AutonSkills:
            // runAutonSkills();
            runAutonSkillsStrategicPush();
            break;
        case autonomousType::Test:
            autonTest();
        default:
            break;
    }
}

namespace {
    void autonTest() {
        setRotation(0.0);
        // driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
        // driveAndTurnDistanceTiles(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
        // driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
        // driveAndTurnDistanceTiles(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
        // driveAndTurnDistanceTiles(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);
        // driveAndTurnDistanceTiles(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 3.0);

        // driveAndTurnDistanceTilesMotionProfile(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(-1.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 1.5);
        // driveAndTurnDistanceTilesMotionProfile(2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);
        // driveAndTurnDistanceTilesMotionProfile(-2.0, 0.0, 100.0, 100.0, defaultMoveTilesErrorRange, 2.0);

        turnToAngle(90);
        turnToAngle(-90);
        turnToAngle(180);
        turnToAngle(-180);
        turnToAngle(0);
    }

    /// @brief Run the 15-seconds red-down autonomous.
    void runAutonRedDown() {
        timer autontimer;
        setRotation(-90.0);

        /* Start facing left */
        
        // 1: Go to middle mobile goal and grab it
        turnToAngle(102.23 - 180.0);
        driveAndTurnDistanceTiles(-2.32, 102.23 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);
        // ...
        return;

        // Score ring
        setIntakeState(1);

        // 2: Intake ring without scoring the stake (bottom of the stack)
        turnToAngle(-105.12);
        driveAndTurnDistanceTiles(0.72, -105.12, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        setIntakeState(0, 0.3);

        // 3: Drop goal at bottom
        turnToAngle(-180.0 + 180.0);
        driveAndTurnDistanceTiles(-0.47, -180.0 + 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        // ...

        // 4-5: Go to left-bottom goal and grab it
        driveAndTurnDistanceTiles(0.58, 0.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        turnToAngle(0.0 + 180.0);
        driveAndTurnDistanceTiles(0.57, 0.0 + 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        // ...

        // Score ring
        setIntakeState(1);

        // 6: Touch the ladder
        turnToAngle(20.81);
        driveAndTurnDistanceTiles(1.01, 20.81, 40.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);
    }

    /// @brief Run the 15-seconds red-up autonomous.
    void runAutonRedUp() {
        timer autontimer;
        setRotation(-90.0);

        /* Start facing left */

        // 1: Go to mobile goal and grab it
        turnToAngle(90.0 - 180.0);
        driveAndTurnDistanceTiles(-1.26, 90.0 - 180.0, 100., 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);

        // Score ring
        setIntakeState(1);

        // 2: Intake ring and score the stake (bottom of the stack)
        turnToAngle(14.0);
        driveAndTurnDistanceTiles(0.97, 14.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);

        // 3: Touch the ladder
        turnToAngle(174.91);
        driveAndTurnDistanceTiles(1.59, 174.91, 40.0, 100.0, autonvals::defaultMoveTilesErrorRange, 3.0);
    }

    /// @brief Run the 15-seconds blue-down autonomous.
    void runAutonBlueDown() {
        
    }

    /// @brief Run the 15-seconds blue-up autonomous.
    void runAutonBlueUp() {
        
    }

    /// @brief Run the skills autonomous.
    void runAutonSkills() {
    }

    void runAutonSkillsCrossBarrier() {
    }

    void runAutonSkillsStrategicPush() {
    }
}