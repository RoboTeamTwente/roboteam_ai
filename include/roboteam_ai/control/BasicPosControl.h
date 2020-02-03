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
        RobotCommand getRobotCommand(world::World *world, const Field *field, const RobotPtr &robot,
                                     const Vector2 &targetPos, const Angle &targetAngle) override;
        RobotCommand getRobotCommand(world::World *world, const Field *field, const RobotPtr &robot,
                                     const Vector2 &targetPos) override;

    };

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_BASICPOSCONTROL_H
