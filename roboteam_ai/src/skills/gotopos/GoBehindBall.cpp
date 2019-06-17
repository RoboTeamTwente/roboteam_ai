//
// Created by baris on 21-2-19.
//

#include "GoBehindBall.h"

namespace rtt {
namespace ai {

GoBehindBall::GoBehindBall(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) {
}

Skill::Status GoBehindBall::gtpUpdate() {

    switch (refType) {
    case penalty: {
        return penaltyUpdate(1);
    }
    case shootOut:{
        return penaltyUpdate(3);
    }
    case freeKick: {
        auto ball = ai::world::world->getBall();
        auto goal = ai::world::field->get_their_goal_center();

        Vector2 v = goal - ball->pos;
        targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS()+0.09)) + ball->pos;
        command.w = (rtt::ai::world::field->getPenaltyPoint(false) - robot->pos).angle();
        if ((targetPos - robot->pos).length2() > errorMargin * errorMargin) {
            return Status::Running;
        }
        else {
            publishRobotCommand();
            return Status::Success;
        }
    }
    case corner:break;
    }
    return Status::Failure;
}

void GoBehindBall::gtpInitialize() {
    posController->setAvoidBallDistance(rtt::ai::Constants::ROBOT_RADIUS() + 0.10);
    if (properties->hasString("type")) {
        refType = stringToRefType(properties->getString("type"));
    }
}

void GoBehindBall::gtpTerminate(Skill::Status s) { }

GoBehindBall::RefType GoBehindBall::stringToRefType(const std::string &string) {
    if (string == "penalty") {
        return penalty;
    }
    else if (string == "corner") {
        return corner;
    }
    else if (string == "freeKick") {
        return freeKick;
    }
    else if (string =="shootOut"){
        return shootOut;
    }
    ROS_ERROR("No string set for the RefType in GoBehindBall Skill!! using freeKick");
    return freeKick;

}
Skill::Status GoBehindBall::penaltyUpdate(int genevaState){
    auto ball = ai::world::world->getBall();
    auto goal = ai::world::field->get_their_goal_center();

    Vector2 v = goal - ball->pos;
    targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS()+0.18)) + ball->pos;
    command.geneva_state = genevaState;
    command.w = (rtt::ai::world::field->get_their_goal_center() - robot->pos).angle();
    return (targetPos - robot->pos).length2() > errorMargin * errorMargin ? Status::Running : Status::Success;

}
}//ai
}//rtt
