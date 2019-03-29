//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H
#define ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "PosController.h"

namespace rtt {
namespace ai {
namespace control {

class ControlGoToPosBallControl : public PosController {
public:
    PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &target) override;
};

} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_CONTROLGOTOPOSBALLCONTROL_H
