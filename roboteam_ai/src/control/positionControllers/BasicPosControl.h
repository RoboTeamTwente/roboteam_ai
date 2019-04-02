//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_BASICPOSCONTROL_H
#define ROBOTEAM_AI_BASICPOSCONTROL_H

#include "PosController.h"

namespace rtt {
namespace ai {
namespace control {

class BasicPosControl : public PosController {
public:
    explicit BasicPosControl() = default;
    explicit BasicPosControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);

    PosVelAngle getPosVelAngle(RobotPtr robot, Vector2 &targetPos) override;
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_BASICPOSCONTROL_H
