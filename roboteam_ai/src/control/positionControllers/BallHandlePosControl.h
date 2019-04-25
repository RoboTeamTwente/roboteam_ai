//
// Created by thijs on 18-12-18.
//

#ifndef ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
#define ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H

#include <roboteam_utils/Vector2.h>
#include "PosController.h"

namespace rtt {
namespace ai {
namespace control {

class BallHandlePosControl : public PosController {
public:
    BallHandlePosControl() = default;
    explicit BallHandlePosControl(bool canMoveInDefenseArea);
    PosVelAngle getPosVelAngle(const RobotPtr &robot, Vector2 &target) override;
private:
    void checkInterfacePID() override;
};



} //control
} //ai
} //rtt

#endif //ROBOTEAM_AI_BALLHANDLEPOSCONTROL_H
