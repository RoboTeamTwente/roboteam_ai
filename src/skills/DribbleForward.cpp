//
// Created by robzelluf on 5/13/19.
//

#include <skills/DribbleForward.h>

namespace rtt::ai {

DribbleForward::DribbleForward(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void DribbleForward::onInitialize() {
    if (properties->hasDouble("dribbleDistance")) {
        dribbleDistance = properties->getDouble("dribbleDistance");
    } else {
        dribbleDistance = 0.9;
    }

    Angle angleToGoal = (field->getTheirGoalCenter() - ball->get()->getPos()).toAngle();
    targetPos = ball->get()->getPos() + Vector2{dribbleDistance, 0}.rotate(angleToGoal);
}

bt::Node::Status DribbleForward::onUpdate() {
    auto c = robot->getControllers().getBallHandlePosController()->getRobotCommand(robot->get()->getId(), targetPos, robot->get()->getAngle(), control::BallHandlePosControl::FORWARDS);

    if (robot->getControllers().getBallHandlePosController()->getStatus() == control::BallHandlePosControl::Status::SUCCESS) {
        return Status::Success;
    }

    command = c.makeROSCommand();
    publishRobotCommand();

    return Status::Running;
}

}  // namespace rtt::ai
