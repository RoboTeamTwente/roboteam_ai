//
// Created by baris on 11-3-19.
//

#include "ShootPenalty.h"

namespace rtt {
namespace ai {

bt::Node::Status ShootPenalty::onUpdate() {
    if (! robot) return Status::Running;

    if (world::field->pointIsInDefenceArea(ball->pos, false)) {
        command.w = robot->angle;
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(ball->pos);
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, false, control::BallSpeed::MAX_SPEED, true, control::ShotPrecision::HIGH);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

}
}