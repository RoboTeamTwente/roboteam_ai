//
// Created by baris on 6-12-18.
//

#ifndef ROBOTEAM_AI_COACH_H
#define ROBOTEAM_AI_COACH_H

#include <roboteam_utils/LastWorld.h>
#include "RobotDealer.h"
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {
namespace coach {

class Coach {
    public:

        using dealer = robotDealer::RobotDealer;
        static int pickOffensivePassTarget(int selfID, std::string roleName);
        static int pickDefensivePassTarget(int selfID);

};

}
}
}

#endif //ROBOTEAM_AI_COACH_H
