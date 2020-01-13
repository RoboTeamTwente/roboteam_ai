//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_BASICPOSCONTROL_H
#define ROBOTEAM_AI_BASICPOSCONTROL_H

#include "PosController.h"

namespace rtt::ai::world {
    class World;
    class Field;
}

namespace rtt::ai::control {

class BasicPosControl : public PosController {
    private:
        void checkInterfacePID() override;
    public:
        BasicPosControl() = default;
        explicit BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);
        RobotCommand getRobotCommand(::rtt::ai::world::World * world,  world::Field * field, const RobotPtr &robot, const Vector2 &targetPos, const Angle &targetAngle) override;
        RobotCommand getRobotCommand(::rtt::ai::world::World * world,  world::Field * field, const RobotPtr &robot, const Vector2 &targetPos) override;

};

} // rtt

#endif //ROBOTEAM_AI_BASICPOSCONTROL_H
