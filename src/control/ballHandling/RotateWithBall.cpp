//
// Created by thijs on 25-5-19.
//

#include "include/roboteam_ai/control/ControlUtils.h"
#include "include/roboteam_ai/control/ballHandling/RotateWithBall.h"
#include "include/roboteam_ai/world/Robot.h"

namespace rtt {
namespace ai {
namespace control {

RobotCommand RotateWithBall::getRobotCommand(const std::shared_ptr<world::Robot> &r, const Vector2 &targetP,
        const Angle &targetA) {

    RobotCommand robotCommand;
    int direction = targetA - r->angle > 0.0 ? 1 : - 1;
    robotCommand.angle = Angle(r->angle + 0.2*direction);
    robotCommand.dribbler = 31;
    return robotCommand;
}

}
}
}