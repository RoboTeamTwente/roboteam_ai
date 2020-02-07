//
// Created by robzelluf on 5/13/19.
//

#include "skills/DribbleForward.h"
#include <world/FieldComputations.h>

namespace rtt::ai {

DribbleForward::DribbleForward(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void DribbleForward::onInitialize() {
    initialBallPos = ball->getPos();
    if (properties->hasDouble("dribbleDistance")) {
        dribbleDistance = properties->getDouble("dribbleDistance");
    } else {
        dribbleDistance = 0.9;
    }

    Angle angleToGoal = ((*field).getTheirGoalCenter() - ball->getPos()).toAngle();
    targetPos = ball->getPos() + Vector2{dribbleDistance, 0}.rotate(angleToGoal);
}

bt::Node::Status DribbleForward::onUpdate() {
    auto c = ballHandlePosControl.getRobotCommand(world, field, robot, targetPos, robot->angle, control::BallHandlePosControl::FORWARDS);

    if (ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    command = c.makeROSCommand();
    publishRobotCommand();

    return Status::Running;
}

}  // namespace rtt::ai
