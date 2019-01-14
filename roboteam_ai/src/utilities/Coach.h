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

namespace rtt {
namespace ai {
namespace coach {

class Coach {

    public:
        static std::map<int, int> defencePairs;
        using dealer = robotDealer::RobotDealer;
        static int pickOffensivePassTarget(int selfID, std::string roleName);
        static int pickDefensivePassTarget(int selfID);
        static int pickOpponentToCover(int selfID);
        static int doesRobotHaveBall(unsigned int robotID, bool isOurTeam);
        static int whichRobotHasBall(bool isOurTeam);
        static int pickHarassmentTarget(int selfID);
        static Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
        static Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
        static Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);

        static bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos);
        static bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition);
        static bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition);

        static std::pair<unsigned int, bool> getRobotClosestToBall();
        static unsigned int getOurRobotClosestToBall();
        static unsigned int getTheirRobotClosestToBall();
};

}
}
}

#endif //ROBOTEAM_AI_COACH_H
