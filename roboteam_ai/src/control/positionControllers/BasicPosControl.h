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
    private:
        void checkInterfacePID() override;
    public:
        BasicPosControl() = default;
        explicit BasicPosControl(bool avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);
        RobotCommand getPosVelAngle(const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) override;
        RobotCommand getPosVelAngle(const RobotPtr &robot, const Vector2 &targetPos) override;

};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_BASICPOSCONTROL_H
