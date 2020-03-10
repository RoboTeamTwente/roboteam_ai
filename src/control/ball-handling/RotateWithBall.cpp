//
// Created by thijs on 25-5-19.
//

#include "control/ball-handling/RotateWithBall.h"
#include "control/ControlUtils.h"
#include "world_new/views/RobotView.hpp"

namespace rtt::ai::control {

RobotCommand RotateWithBall::getRobotCommand(const world_new::view::RobotView r, const Vector2 &targetP, const Angle &targetA) {
    RobotCommand robotCommand;
    int direction = targetA - r->getAngle() > 0.0 ? 1 : -1;
    robotCommand.angle = Angle(r->getAngle() + 0.2 * direction);
    robotCommand.dribbler = 31;
    return robotCommand;
}

}  // namespace rtt::ai::control
