//
// Created by robzelluf on 5/13/19.
//

#include <roboteam_ai/src/world/Field.h>
#include "DribbleForward.h"

namespace rtt {
namespace ai {

DribbleForward::DribbleForward(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {}

void DribbleForward::onInitialize() {
    initialBallPos = ball->pos;
    if (properties->hasDouble("dribbleDistance")) {
        dribbleDistance = properties->getDouble("dribbleDistance");
    } else {
        dribbleDistance = 0.9;
    }

    Angle angleToGoal = (world::field->get_their_goal_center() - ball->pos).toAngle();
    targetPos = ball->pos + Vector2{dribbleDistance, 0}.rotate(angleToGoal);
}


bt::Node::Status DribbleForward::onUpdate() {

    auto c = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle, control::BallHandlePosControl::FORWARDS);

    if (ballHandlePosControl.getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    command = c.makeROSCommand();
    publishRobotCommand();

    return Status::Running;
}

} //ai
} //rtt
