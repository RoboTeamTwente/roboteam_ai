//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_FORCEPOSCONTROL_H
#define ROBOTEAM_AI_FORCEPOSCONTROL_H

#include "PosController.h"

namespace rtt {
namespace ai {
namespace control {

class ForcePosControl : public PosController {
    explicit ForcePosControl() = default;
    PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) override;
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_FORCEPOSCONTROL_H
