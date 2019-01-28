//
// Created by baris on 15-1-19.
//

#include "BasicGoToPos.h"

namespace rtt {
namespace ai {

BasicGoToPos::BasicGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {

}

void BasicGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);
    if (properties->hasVector2("target")) targetPos = properties->getVector2("target");
    if (properties->getBool("goToBall")) targetPos = ball->pos;
    goToPos.setAvoidBall(properties->getBool("avoidBall"));
    goToPos.setCanGoOutsideField(properties->getBool("canGoOutsideField"));
    targetPos = properties->getVector2("target");
    if (properties->getBool("mirror")) targetPos = {robot->pos.x, -robot->pos.y};

    if (properties->getBool("BallPlacementBefore")){
        if(ball){
            targetPos=coach::Coach::getBallPlacementBeforePos(ball->pos);
        }
        else{
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    else if (properties->getBool("BallPlacementAfter")){
        if(ball){
            errorMargin=0.05;
            targetPos=coach::Coach::getBallPlacementAfterPos(ball->pos,robot->angle);
        }
        else{
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
}


Skill::Status BasicGoToPos::onUpdate() {

    if (! robot) return Status::Running;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    if(properties->getBool("BallPlacementAfter")){
        command.w=static_cast<float>((Vector2(robot->pos)-targetPos).angle());
    }
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::luTh);
    velocity = control::ControlUtils::VelocityLimiter(velocity);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    publishRobotCommand(command);

    Vector2 deltaPos = targetPos - robot->pos;

    if (deltaPos.length() > errorMargin) {
        return Status::Running;
    }
    else {
        return Status::Success;
    }
}

}
}
