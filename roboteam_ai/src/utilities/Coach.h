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
};

}
}
}

#endif //ROBOTEAM_AI_COACH_H
