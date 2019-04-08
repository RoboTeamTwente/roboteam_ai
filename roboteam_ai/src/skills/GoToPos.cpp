//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"
#include "roboteam_ai/src/world/Field.h"
#include "../coach/GeneralPositionCoach.h"
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

        gotopos.setAvoidBall(properties->getBool("avoidBall"));
        gotopos.setCanMoveOutOfField(properties->getBool("canGoOutsideField"));
}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {

    if ((targetPos - robot->pos).length() < errorMargin) {
        return Status::Success;
    }

    control::PosVelAngle pva = gotopos.getPosVelAngle(robot, targetPos);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, maxVel);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    command.w = static_cast<float>(pva.angle);

    publishRobotCommand();
    return Status::Running;
}

void GoToPos::onTerminate(Status s) {
    command.w = robot->angle;
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand();
}

} // ai
} // rtt