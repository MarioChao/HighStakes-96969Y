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
    autonomousType auton_runType = autonomousType::BlueUp;
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
            runAutonRedUp();
            break;
        case autonomousType::RedDown:
            runAutonRedDown();
            break;
        case autonomousType::BlueUp:
            runAutonBlueUp();
            break;
        case autonomousType::BlueDown:
            runAutonBlueDown();
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
        driveAndTurnDistanceTiles(-1.82, 102.23 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);
        driveAndTurnDistanceTiles(-0.50, 102.23 - 180.0, 25.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        // ...
        setGoalClampState(1);

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
        setGoalClampState(0);

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
        driveAndTurnDistanceTiles(-0.76, 90.0 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);
        driveAndTurnDistanceTiles(-0.5, 90.0 - 180.0, 25.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);

        setGoalClampState(1);

        // Score ring
        setIntakeState(1);

        // 2: Intake ring and score the stake (bottom of the stack)
        turnToAngle(14.0);
        driveAndTurnDistanceTiles(0.97, 14.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);

        setGoalClampState(0);

        // 3: Touch the ladder
        turnToAngle(174.91);
        driveAndTurnDistanceTiles(1.59, 174.91, 40.0, 100.0, autonvals::defaultMoveTilesErrorRange, 3.0);
    }

    /// @brief Run the 15-seconds blue-down autonomous.
    void runAutonBlueDown() {
        timer autontimer;
        setRotation(90.0);

        /* Start facing left */
        
        // 1: Go to middle mobile goal and grab it
        turnToAngle(-102.23 + 180.0);
        driveAndTurnDistanceTiles(-1.82, -102.23 + 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);
        driveAndTurnDistanceTiles(-0.50, -102.23 + 180.0, 25.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        // ...
        setGoalClampState(1);

        // Score ring
        setIntakeState(1);

        // 2: Intake ring without scoring the stake (bottom of the stack)
        turnToAngle(105.12);
        driveAndTurnDistanceTiles(0.72, 105.12, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        setIntakeState(0, 0.3);

        // 3: Drop goal at bottom
        turnToAngle(+180.0 - 180.0);
        driveAndTurnDistanceTiles(-0.47, +180.0 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        // ...
        setGoalClampState(0);

        // 4-5: Go to left-bottom goal and grab it
        driveAndTurnDistanceTiles(0.58, 0.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        turnToAngle(0.0 - 180.0);
        driveAndTurnDistanceTiles(0.57, 0.0 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        // ...

        // Score ring
        setIntakeState(1);

        // 6: Touch the ladder
        turnToAngle(-20.81);
        driveAndTurnDistanceTiles(1.01, -20.81, 40.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);
    }

    /// @brief Run the 15-seconds blue-up autonomous.
    void runAutonBlueUp() {
         timer autontimer;
        setRotation(90.0);

        /* Start facing left */

        /*
        // 1: Go to mobile goal and grab it

        turnToAngle(-113.19 + 180.0);
        driveAndTurnDistanceTiles(-1.02, -113.19 + 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);
        driveAndTurnDistanceTiles(-0.42, -113.19 + 180.0, 30.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);
        setGoalClampState(1);

        // 2: Bump into ladder

        turnToAngle(-159.79 + 180.0);
        driveAndTurnDistanceTiles(-0.71, -159.79 + 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        
        // 3: Go prepare intake

        turnToAngle(-40.41);
        driveAndTurnDistanceTiles(1.27, -40.41, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);

        // 4: Intake two rings

        setIntakeState(1);
        turnToAngle(0.74);
        driveAndTurnDistanceTiles(0.79, 0.74, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);

        // 5: Intake one ring halfway

        turnToAngle(105.04);
        driveAndTurnDistanceTiles(0.75, 105.04, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);
        setIntakeState(0, 0.3);

        // Drop goal

        setGoalClampState(0);

        // 6: Go to wall stake

        turnToAngle(139.52);
        driveAndTurnDistanceTiles(2.09, 139.52, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 2.0);

        // 7: Turn around and push ring away

        turnToAngle(145.14 - 180.0);
        driveAndTurnDistanceTiles(-0.53, 145.14 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);

        // 8: Prepare to score wall stake

        turnToAngle(90.0 - 180.0);
        driveAndTurnDistanceTiles(-0.18, 90.0 - 180.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange, 1.5);

        // Score wall stake

        setIntakeState(1);
        setIntakeState(0, 1.5);/**/
       
        
        driveAndTurnDistanceTiles(-0.225, 90.0, 100.0, 100.0, autonvals::defaultMoveTilesErrorRange/2, 0.75);
        driveAndTurnDistanceTiles(-0.22, 45.0, 100.0, 100.0, 0, 1);
        setGoalClampState(1);
        driveAndTurnDistanceTiles(-0.18, 0.0, 60., 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        driveAndTurnDistanceTiles(0.17, -30.0, 56., 80.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        setIntakeState(1);
        driveAndTurnDistanceTiles(0.64, 0.0, 30., 100.0, autonvals::defaultMoveTilesErrorRange, 1.4);
        driveAndTurnDistanceTiles(-0.165, -20.0, 30.*10/7, 70.0*10/7, autonvals::defaultMoveTilesErrorRange, 1.0);
        turnToAngleVelocity(45, 100, 0, autonvals::defaultTurnAngleErrorRange, 1);
        driveAndTurnDistanceTiles(0.1, 45.0, 60., 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        turnToAngleVelocity(150, 100, 0, autonvals::defaultTurnAngleErrorRange, 1);
        setGoalClampState(0);
        IntakeLiftPneumatic.set(1);
        driveAndTurnDistanceTiles(0.5, 145.0, 100., 100.0, autonvals::defaultMoveTilesErrorRange, 1.0);
        IntakeLiftPneumatic.set(0);
        setIntakeState(0);
        turnToAngleVelocity(270,100, -1, autonvals::defaultTurnAngleErrorRange, 1);
        driveAndTurnDistanceTiles(-0.1, 270.0, 100, 100, autonvals::defaultMoveTilesErrorRange, 1.0);
        setIntakeState(1);/**/

    }

    /// @brief Run the skills autonomous.
    void runAutonSkills() {
    }

    void runAutonSkillsCrossBarrier() {
    }

    void runAutonSkillsStrategicPush() {
    }
}