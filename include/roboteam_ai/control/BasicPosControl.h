//
// Created by mrlukasbos on 27-3-19.
//

#ifndef ROBOTEAM_AI_BASICPOSCONTROL_H
#define ROBOTEAM_AI_BASICPOSCONTROL_H

#include "PosController.h"

namespace rtt::ai::control {

class BasicPosControl : public PosController {
   private:
    void checkInterfacePID() override;

   public:
    BasicPosControl() = default;
    explicit BasicPosControl(double avoidBall, bool canMoveOutsideField, bool canMoveInDefenseArea);
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const world::Field *field, const world_new::view::RobotView &robot, const Vector2 &targetPos, const Angle &targetAngle) override;
    RobotCommand getRobotCommand(world_new::view::WorldDataView *world, const world::Field *field, const world_new::view::RobotView &robot, const Vector2 &targetPos) override;
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_BASICPOSCONTROL_H
