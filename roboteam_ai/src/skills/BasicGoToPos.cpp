//
// Created by baris on 15-1-19.
//

#include <roboteam_ai/src/coach/Ballplacement.h>
#include <roboteam_ai/src/coach/GeneralPositionCoach.h>
#include "BasicGoToPos.h"
#include <roboteam_ai/src/utilities/Field.h>

namespace rtt {
namespace ai {

BasicGoToPos::BasicGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

void BasicGoToPos::onInitialize() {
    targetPos = properties->getVector2("target");
    if (properties->getBool("BallPlacementBefore")) {
        if (ball) {
            //TODO:Changed for testing, remember to change back
            //targetPos=coach::Coach::getBallPlacementBeforePos(ball->pos);
            targetPos = coach::g_ballPlacement.getBallPlacementPos();
        }
        else {
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    else if (properties->getBool("BallPlacementAfter")) {
        if (ball) {
            errorMargin = 0.05;
            targetPos = coach::g_ballPlacement.getBallPlacementAfterPos(robot->angle);
        }
        else {
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    else if (properties->getBool("DemoKeeperGetBall")){
        if(ball){
            errorMargin=0.05;
            targetPos=coach::g_generalPositionCoach.getDemoKeeperGetBallPos(ball->pos);
        }
        else{
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    if (properties->getBool("goToBall")) targetPos = ball->pos;

    if (properties->getBool("getBallFromSide")) targetPos = getBallFromSideLocation();

    if (properties->getDouble("maxVel")) {
        maxVel = properties->getDouble("maxVel");
    } else maxVel = Constants::MAX_VEL();

    goToPos.setAvoidBall(properties->getBool("avoidBall"));
    goToPos.setCanGoOutsideField(properties->getBool("canGoOutsideField"));
}

Skill::Status BasicGoToPos::onUpdate() {
    if (! robot) return Status::Running;
//    if (properties->getBool("BallPlacementAfter")){
//        targetPos=coach::Coach::getBallPlacementAfterPos(robot->angle);
//    }
    else if(properties->getBool("BallPlacementBefore")){
        targetPos=coach::g_ballPlacement.getBallPlacementBeforePos(ball->pos);
    }
    else if(properties->getBool("DemoKeeperGetBall")){
        targetPos=coach::g_generalPositionCoach.getDemoKeeperGetBallPos(ball->pos);
    }

    if (properties->getBool("getBallFromSide")) targetPos = getBallFromSideLocation();

    if (!Field::pointIsInField(targetPos)) return Status::Failure;

    Vector2 deltaPos = (targetPos - robot->pos);

    if (deltaPos.length() < errorMargin) {
        return Status::Success;
    }
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::PosControlType::NUMERIC_TREES).vel;
    if(properties->getBool("BallPlacementAfter")){
        command.w=static_cast<float>((Vector2(robot->pos)-targetPos).angle());
        velocity = goToPos.goToPos(robot, targetPos, control::PosControlType::BASIC).vel;
    }

    velocity=control::ControlUtils::VelocityLimiter(velocity,maxVel,0.3);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);
    return Status::Running;
}

Vector2 BasicGoToPos::getBallFromSideLocation() {
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double distanceFromTop      = abs(field.field_width / 2 - ball->pos.y);
    double distanceFromBottom   = abs(-field.field_width / 2 - ball->pos.y);
    double distanceFromLeft     = abs(-field.field_length / 2 - ball->pos.x);
    double distanceFromRight    = abs(field.field_length / 2 - ball->pos.x);

    double distance = 999;
    Vector2 pos;
    if (distanceFromTop < distance) {
        distance = distanceFromTop;
        pos = {ball->pos.x, ball->pos.y - getballFromSideMargin};
    }
    if (distanceFromBottom < distance) {
        distance = distanceFromBottom;
        pos = {ball->pos.x, ball->pos.y + getballFromSideMargin};
    }

    if (distance < 0.20) {
        return pos;
    }
    if (distanceFromLeft < distance) {
        distance = distanceFromLeft;
        pos = {ball->pos.x + getballFromSideMargin, ball->pos.y};
    }
    if (distanceFromRight < distance) {
        pos = {ball->pos.x - getballFromSideMargin, ball->pos.y};
    }

    return pos;
}

void BasicGoToPos::onTerminate(Status s) {
    if (properties->hasDouble("angle")) {
        roboteam_msgs::RobotCommand command;
        command.id = robot->id;
        double targetAngle = control::ControlUtils::constrainAngle(properties->getDouble("angle") * M_PI);
        command.w = static_cast<float>(targetAngle);
        publishRobotCommand(command);
    }
}

}
}
