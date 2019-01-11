//
// Created by baris on 6-12-18.
//

#ifndef ROBOTEAM_AI_COACH_H
#define ROBOTEAM_AI_COACH_H

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
        static rtt::Vector2 getPositionBehindBall(double distanceBehindBall);
        };

}
}
}

#endif //ROBOTEAM_AI_COACH_H
