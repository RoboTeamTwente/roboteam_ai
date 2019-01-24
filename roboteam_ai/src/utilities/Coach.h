//
// Created by baris on 6-12-18.
//

#ifndef ROBOTEAM_AI_COACH_H
#define ROBOTEAM_AI_COACH_H

#include <roboteam_utils/LastWorld.h>
#include "RobotDealer.h"
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/dangerfinder/DangerData.h>
#include <roboteam_ai/src/dangerfinder/DangerFinder.h>
#include <roboteam_ai/src/dangerfinder/DangerFinder.h>
#include "../control/ControlUtils.h"
#include <map>
#include <string>
#include "roboteam_utils/Vector2.h"

namespace rtt {
namespace ai {
namespace coach {

class Coach {
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
    static int doesRobotHaveBall(unsigned int robotID, bool isOurTeam);
    static int doesRobotHaveBall(unsigned int robotID, bool isOurTeam, double dist);
    static int doesRobotHaveBall(unsigned int robotID, bool isOurTeam, double dist, double angle);

    static int whichRobotHasBall(bool isOurTeam);
    static int pickHarassmentTarget(int selfID);
    static Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
    static Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
    static Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);
    static Vector2 getRobotPositionClosestToPositionPosition(std::vector<roboteam_msgs::WorldRobot> &robots,
                                                             Vector2 position, bool includeSamePosition);


    static bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos);
    static bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition);
    static bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition);

    static std::pair<int, bool> getRobotClosestToBall();
    static int getOurRobotClosestToBall();
    static int getTheirRobotClosestToBall();

    static int getRobotClosestToGoal(bool ourRobot, bool ourGoal);

    static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToBall(bool isOurTeam);

    static void addDefender(int id);
    static void removeDefender(int id);
    static Vector2 getDefensivePosition(int robotId);

    // Pass variables and functions
    static bool initiatePass(int robotID);
    static bool isReadyToReceivePass();
    static void setReadyToReceivePass(bool readyToReceivePass);
    static int getRobotBeingPassedTo();
    static void setRobotBeingPassedTo(int robotBeingPassedTo);
    static bool isPassed();
    static void setPassed(bool passed);

    static void addFormationRobot(int id);
    static void removeFormationRobot(int id);
    static Vector2 getFormationPosition(int robotId);

};

}
}
}

#endif //ROBOTEAM_AI_COACH_H
