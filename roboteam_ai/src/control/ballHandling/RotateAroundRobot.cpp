//
// Created by thijs on 25-5-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>

#include "RotateAroundRobot.h"

namespace rtt {
namespace ai {
namespace control {

RobotCommand RotateAroundRobot::getRobotCommand(const world::Robot::RobotPtr &r, const Vector2 &targetP,
        const Angle &targetA) {

    RobotCommand robotCommand;
    int direction = targetAngle - robot->angle > 0.0 ? 1 : - 1;
    robotCommand.angle = Angle(robot->angle + 0.2*direction);
    robotCommand.dribbler = 1;
    return robotCommand;
}

}
}
}