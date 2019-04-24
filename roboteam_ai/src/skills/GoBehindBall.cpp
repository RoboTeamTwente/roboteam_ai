//
// Created by baris on 21-2-19.
//

#include "GoBehindBall.h"

namespace rtt {
namespace ai {

GoBehindBall::GoBehindBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
    goToPos.setAvoidBall(0.02);
}

Skill::Status GoBehindBall::onUpdate() {
    switch (unitType) {
        case penalty: {
            auto ball = ai::world::world->getBall();
            auto goal = ai::world::field->get_their_goal_center();

            Vector2 v = goal - ball->pos;
            auto targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS())) + ball->pos;
            // TODO draw the point in the interface

            Vector2 velocity = goToPos.getPosVelAngle(robot, targetPos).vel;

            Vector2 deltaPos = targetPos - robot->pos;
            publishCommand(targetPos, velocity);

            if (deltaPos.length() > errorMargin) {
                return Status::Running;
            }
            else {



                return Status::Success;
            }
        }

        case freeKick: {

            // TODO optimize for free kick

            auto ball = ai::world::world->getBall();
            auto goal = ai::world::field->get_their_goal_center();

            Vector2 v = goal - ball->pos;
            auto targetPos = ((v*- 1.0).stretchToLength(rtt::ai::Constants::ROBOT_RADIUS())) + ball->pos;
            // TODO draw the point in the interface

            Vector2 velocity = goToPos.getPosVelAngle(robot, targetPos).vel;

            Vector2 deltaPos = targetPos - robot->pos;
            publishCommand(targetPos, velocity);

            if (deltaPos.length() > errorMargin) {
                return Status::Running;
            }
            else {
                command.w = robot->angularVelocity;
                command.x_vel = 0;
                command.y_vel =0;
                command.geneva_state = 1;
                publishRobotCommand();
                return Status::Success;
            }
        }
        case corner:
            break;
    }
    return Status::Failure;
}

void GoBehindBall::onInitialize() {
    if (properties->hasString("type")) {
        unitType = stringToUnit(properties->getString("type"));
    }
    goToPos.setAvoidBall(0.10);
}

void GoBehindBall::onTerminate(Skill::Status s) {

}

GoBehindBall::unit GoBehindBall::stringToUnit(std::string string) {
    if (string == "penalty") {
        return penalty;
    }
    else if (string == "corner") {
        return corner;
    }
    else {
        return freeKick;
    }
}
void GoBehindBall::publishCommand(Vector2 targetPos, Vector2 velocity) {
    command.w = static_cast<float>((targetPos - robot->pos).angle());
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);

    if (unitType == penalty)
        command.geneva_state = 1;

    publishRobotCommand();

}
}
}