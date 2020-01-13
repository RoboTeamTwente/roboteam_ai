//
// Created by thijs on 25-5-19.
//

#include "control/ControlUtils.h"
#include "control/ballHandling/RotateWithBall.h"
#include "world_old/Robot.h"

namespace rtt::ai::control {

RobotCommand RotateWithBall::getRobotCommand(const std::shared_ptr<world::Robot> &r, const Vector2 &targetP,
        const Angle &targetA) {

    RobotCommand robotCommand;
    int direction = targetA - r->angle > 0.0 ? 1 : - 1;
    robotCommand.angle = Angle(r->angle + 0.2*direction);
    robotCommand.dribbler = 31;
    return robotCommand;
}

}