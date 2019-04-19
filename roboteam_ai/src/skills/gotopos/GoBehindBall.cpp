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
        auto ball = ai::world::world->getBall();
        auto goal = ai::world::field->get_their_goal_center();

        Vector2 v = goal - ball->pos;
        targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS())) + ball->pos;
        command.geneva_state = 1;
        return (targetPos - robot->pos).length2() > errorMargin * errorMargin ? Status::Running : Status::Success;
    }
    case freeKick: {

        // TODO optimize for free kick

        auto ball = ai::world::world->getBall();
        auto goal = ai::world::field->get_their_goal_center();

        Vector2 v = goal - ball->pos;
        targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS())) + ball->pos;
        // TODO draw the point in the interface
        if ((targetPos - robot->pos).length2() > errorMargin * errorMargin) {
            return Status::Running;
        }
        else {
            command.w = robot->angularVelocity;
            command.x_vel = 0;
            command.y_vel = 0;
            command.geneva_state = 1;
            publishRobotCommand();
            return Status::Success;
        }
    }
    case corner:break;
    }
    return Status::Failure;
}

void GoBehindBall::gtpInitialize() {
    posController->setAvoidBall(0.02);
    if (properties->hasString("type")) {
        refType = stringToRefType(properties->getString("type"));
    }
}

void GoBehindBall::gtpTerminate(Skill::Status s) {
}

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
    ROS_ERROR("No string set for the RefType in GoBehindBall Skill!! using freeKick");
    return freeKick;

}

}
}
