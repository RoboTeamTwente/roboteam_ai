#include <utility>

#include <utility>

//
// Created by thijs on 26-4-19.
//

#include "GTPWithBall.h"

namespace rtt {
namespace ai {

GTPWithBall::GTPWithBall(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) { }

void GTPWithBall::gtpInitialize() {
    posController = std::make_shared<control::BallHandlePosControl>();
    targetType = stringToTargetType(properties->getString("targetType"));
    updateTarget();
}

Skill::Status GTPWithBall::gtpUpdate() {
    if (!robot->hasBall()) return Status::Failure;
    updateTarget();
    return Status::Running;
}

void GTPWithBall::gtpTerminate(Skill::Status s) {

}

GTPWithBall::TargetType GTPWithBall::stringToTargetType(const std::string &string) {
    return rotateToTheirGoal;
}

void GTPWithBall::updateTarget() {
    switch (targetType) {
    default:return;
    case rotateToTheirGoal: {
        Vector2 targetVector = world::field->get_their_goal_center() - robot->pos;
        targetPos = robot->pos;
        targetAngle = targetVector.toAngle();
        return;
    }
    }
}

}
}