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

    public:
        static std::map<int, int> defencePairs;
        static int pickOffensivePassTarget(int selfID, std::string roleName);
        static int pickDefensivePassTarget(int selfID);
        static int pickOpponentToCover(int selfID);
        static int doesRobotHaveBall(unsigned int robotID, bool isOurTeam);
        static int whichRobotHasBall(bool isOurTeam);
        static int pickHarassmentTarget(int selfID);
        static Vector2 getPositionBehindBallToGoal(double distanceBehindBall, bool ourGoal);
        static Vector2 getPositionBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID);
        static Vector2 getPositionBehindBallToPosition(double distanceBehindBall, const Vector2 &position);
        static Vector2 getRobotClosestToPosition(std::vector<roboteam_msgs::WorldRobot> &robots, Vector2 position, bool includeSamePosition);


        static bool isRobotBehindBallToGoal(double distanceBehindBall, bool ourGoal, const Vector2 &robotPos);
        static bool isRobotBehindBallToRobot(double distanceBehindBall, bool ourRobot, const unsigned int &robotID, const Vector2 &robotPosition);
        static bool isRobotBehindBallToPosition(double distanceBehindBall, const Vector2 &position, const Vector2 &robotPosition);

        static std::pair<int, bool> getRobotClosestToBall();
        static int getOurRobotClosestToBall();
        static int getTheirRobotClosestToBall();

        enum PassState {
          goBehindBall,
          goToReceiveBall,
          shoot,
          receive,
        };
        static std::map<std::string,PassState> passState;
        static PassState getPassState(std::string role);


};

}
}
}

#endif //ROBOTEAM_AI_COACH_H
