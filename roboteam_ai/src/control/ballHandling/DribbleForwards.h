//
// Created by thijs on 25-5-19.
//

#ifndef ROBOTEAM_AI_DRIBBLEFORWARDS_H
#define ROBOTEAM_AI_DRIBBLEFORWARDS_H

#include <roboteam_utils/Angle.h>
#include <roboteam_utils/Vector2.h>
#include "roboteam_ai/src/control/positionControllers/RobotCommand.h"
#include "roboteam_ai/src/world/Robot.h"

namespace rtt {
namespace ai {
namespace control {

class DribbleForwards {
    public:
        enum ForwardsProgress : short {
          F_start,
          F_turning,
          F_approaching,
          F_dribbleForward,
          F_success,
          F_fail
        };
        ForwardsProgress getForwardsProgression();

    private:
        ForwardsProgress forwardsProgress = F_start;
        void printForwardsProgress();

        // variables for forwards progress
        std::pair<Vector2, Vector2> F_forwardsDribbleLine;
        Angle lockedAngle;

        // functions for forwards progress
        void updateForwardsProgress();
        RobotCommand sendForwardsCommand();
        RobotCommand F_startTravelForwards();
        RobotCommand F_sendTurnCommand();
        RobotCommand F_sendApproachCommand();
        RobotCommand F_sendDribbleForwardsCommand();

    public:
        RobotCommand getRobotCommand(const world::Robot::RobotPtr &r,
                const Vector2 &targetP, const Angle &targetA);
        void reset();
        DribbleForwards() = default;
};

}
}
}

#endif //ROBOTEAM_AI_DRIBBLEFORWARDS_H
