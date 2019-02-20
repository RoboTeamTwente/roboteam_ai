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
    targetPos = properties->getVector2("target");
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
            //targetPos=coach::Coach::getBallPlacementAfterPos(robot->angle);
            double targetAngle = control::ControlUtils::constrainAngle(robot->angle - M_PI);
            targetPos = (Vector2){0.2, 0}.rotate(targetAngle);
        }
        else{
            ROS_ERROR("BasicGoToPos: No ball found! assuming (%f,%f)", targetPos.x, targetPos.y);
        }
    }
    else if (properties->getBool("DemoKeeperGetBall")){
        if(ball){
            errorMargin=0.05;
            targetPos=coach::Coach::getDemoKeeperGetBallPos(ball->pos);
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
    if (properties->getBool("BallPlacementAfter")){
        targetPos=coach::Coach::getBallPlacementAfterPos(robot->angle);
    }
    else if(properties->getBool("BallPlacementBefore")){
        targetPos=coach::Coach::getBallPlacementBeforePos(ball->pos);
    }
    else if(properties->getBool("DemoKeeperGetBall")){
        targetPos=coach::Coach::getDemoKeeperGetBallPos(ball->pos);
    }

    Vector2 deltaPos = targetPos - robot->pos;

    if (deltaPos.length() < errorMargin) {
        return Status::Success;
    }
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    if(properties->getBool("BallPlacementAfter")){
        command.w=static_cast<float>((Vector2(robot->pos)-targetPos).angle());
    }
//    const ros::Time &t1 = ros::Time::now();
    Vector2 velocity = goToPos.goToPos(robot, targetPos, control::GoToType::luTh);
//    const ros::Time &t2 = ros::Time::now();
//    std::cerr << "gotopos took: " << (t2-t1).toNSec()*0.000001 << " ms" << std::endl;
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
