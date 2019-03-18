//
// Created by baris on 6-12-18.
//

#ifndef ROBOTEAM_AI_COACH_H
#define ROBOTEAM_AI_COACH_H

#include <roboteam_utils/LastWorld.h>
#include "RobotDealer.h"
#include <roboteam_ai/src/control/ControlUtils.h>
#include "../control/ControlUtils.h"
#include <map>
#include <string>
#include <gtest/gtest_prod.h>
#include "roboteam_utils/Vector2.h"
#include "../interface/InterfaceValues.h"

namespace rtt {
namespace ai {
namespace coach {

class Coach {
    FRIEND_TEST(CoachTest, it_adds_and_removes_defenders);
    FRIEND_TEST(CoachTest, it_adds_and_removes_formationrobots);

private:
        static std::vector<int> defenders;
        static std::vector<int> robotsInFormation;

    // Pass variables
    static bool readyToReceivePass;
    static int robotBeingPassedTo;
    static bool passed;
public:
    static std::map<int, int> defencePairs;
    static int pickOffensivePassTarget(int selfID, std::string roleName);
    static int pickDefensivePassTarget(int selfID);
    static int pickOpponentToCover(int selfID);
    static int pickHarassmentTarget(int selfID);
    static Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
    static Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
    static Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);
    static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPosition(
            std::vector<roboteam_msgs::WorldRobot> &robots,
            Vector2 position, bool includeSamePosition);


    static bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos);
    static bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition);
    static bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition);

    static std::pair<int, bool> getRobotClosestToBall();

    static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToBall(bool isOurTeam);

    static void addDefender(int id);
    static void removeDefender(int id);
    static Vector2 getDefensivePosition(int robotId);

    // Pass variables and functions
    static void resetPass();
    static int initiatePass();
    static bool isReadyToReceivePass();
    static void setReadyToReceivePass(bool readyToReceivePass);
    static int getRobotBeingPassedTo();
    static void setRobotBeingPassedTo(int robotBeingPassedTo);
    static bool isPassed();
    static void setPassed(bool passed);

    static void addFormationRobot(int id);
    static void removeFormationRobot(int id);
    static Vector2 getFormationPosition(int robotId);

    static Vector2 getBallPlacementPos();
    static Vector2 getBallPlacementBeforePos(Vector2 ballPos);
    static Vector2 getBallPlacementAfterPos(double RobotAngle);
    static Vector2 getDemoKeeperGetBallPos(Vector2 ballPos);
};

}
}
}

#endif //ROBOTEAM_AI_COACH_H
