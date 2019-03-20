//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"
#include "../utilities/Field.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

/// Init the GoToPos skill
void GoToPos::onInitialize() {
    if (properties->hasVector2("targetPos")) {
        targetPos = properties->getVector2("targetPos");
    }

    if (properties->hasDouble("maxVel")) {
        maxVel = properties->getDouble("maxVel");
    }

    goToPos.setAvoidBall(properties->getBool("avoidBall"));
    goToPos.setCanGoOutsideField(properties->getBool("canGoOutsideField"));
}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {
    if (! robot) return Status::Running;

    if ((targetPos - robot->pos).length() < errorMargin) {
        return Status::Success;
    }

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;

    control::PosVelAngle pva = goToPos.goToPos(robot, targetPos, control::PosControlType::NUMERIC_TREES);
    pva.vel = control::ControlUtils::VelocityLimiter(pva.vel);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    command.w = static_cast<float>(pva.angle);

    publishRobotCommand(command);
    return Status::Running;
}

void GoToPos::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = robot->angle;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

} // ai
} // rtt