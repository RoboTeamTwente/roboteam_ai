//
// Created by thijs on 18-12-18.
//

#include "GoToPosInclude.h"

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H
#define ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H

namespace rtt {
namespace ai {
namespace control {

class ControlGoToPosBallControl {
    private:
        using RobotPtr = std::shared_ptr<roboteam_msgs::WorldRobot>;

    public:
        Vector2 goToPos(RobotPtr robot, Vector2 &target);
};

} //control
} //ai
} //rtt
#endif //ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H
