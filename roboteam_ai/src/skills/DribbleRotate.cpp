#include <utility>

#include <utility>

//
// Created by rolf on 14/12/18.
//
// TODO: Test real robot rotation speeds.
// TODO: Make the robot automatically slow down/speed up if the ball is going to one end of the dribbler. Control?
#include "DribbleRotate.h"
#include "../control/ControlUtils.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
DribbleRotate::DribbleRotate(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void DribbleRotate::checkProgression() {
    double angDif = Control::angleDifference(robot->angle, targetAngle);
    if (!robotHasBall(constants::MAX_BALL_BOUNCE_RANGE)){ // change to !botHassball() alter
        currentProgression=FAIL;
        return;
    }
    if (angDif<0.1*M_PI && currentTick>=maxTick+extraTick){
        currentProgression=SUCCESS;
        return;
    }
    else{
        currentProgression=ROTATING;
    }
}
void DribbleRotate::onInitialize() {
    if (properties->hasDouble("maxVel")) {
        maxSpeed = properties->getDouble("maxVel");
    }
    else {
        maxSpeed = constants::DRIBBLE_ROTATE_MAX_SPEED;
    }
    if (properties->hasDouble("Angle")) {
        targetAngle = properties->getDouble("Angle");
    }
    else if (properties->getBool("RotateToTheirGoal")){
        Vector2 theirCentre=Field::get_their_goal_center();
        targetAngle=(theirCentre-robot->pos).angle();
    }
    if (!properties->hasDouble("Angle")&&!properties->hasBool("RotateToTheirGoal")){
        ROS_ERROR(" dribbleRotate Initialize -> No good angle set in properties");
        currentProgression = FAIL;
    }
    startAngle = robot->angle;
    incrementAngle= maxSpeed/constants::tickRate;
    currentProgression=ROTATING;
    currentTick=0;
    extraTick= static_cast<int>(constants::DRIBBLE_ROTATE_WAIT_TIME*constants::tickRate);
    dir=Control::rotateDirection(startAngle,targetAngle);
    maxTick=(int)floor(Control::angleDifference(startAngle,targetAngle)/maxSpeed*constants::tickRate);
    if (!robotHasBall(constants::MAX_BALL_RANGE)){
        std::cout<<"Robot does not have ball in dribbleRotateInitialize"<<std::endl;
        std::cout<< "Distance"<<(Vector2(robot->pos)-Vector2(ball->pos)).length()-constants::ROBOT_RADIUS<<" Max distance:"<<constants::MAX_BALL_RANGE<<std::endl;
        currentProgression=FAIL;
        std::cout<<robot->angle<<std::endl;
    }
    else {
        std::cout << "Robot has ball in dribbleRotate Initialize" << std::endl;
    }
}
DribbleRotate::Status DribbleRotate::onUpdate() {
    checkProgression();
    switch (currentProgression){
    case ROTATING: sendMoveCommand();
        return Status::Running;
    case SUCCESS:
        return Status::Success;
    case FAIL:
        return Status::Failure;
    }
    return Status::Failure;
}

void DribbleRotate::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.w = static_cast<float>(targetAngle);
    publishRobotCommand(command);
}
void DribbleRotate::sendMoveCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.w = static_cast<float>(computeCommandAngle());
    currentTick++;
    publishRobotCommand(command);
}
double DribbleRotate::computeCommandAngle() {
    if (currentTick<maxTick){
        return Control::constrainAngle(startAngle+dir*currentTick*incrementAngle);
    }
    else return targetAngle;
}
bool DribbleRotate::robotHasBall(double frontRange) {
    //The ball is in an area defined by a cone from the robot centre, or from a rectangle in front of the dribbler
    Vector2 RobotPos = robot->pos;
    Vector2 BallPos = ball->pos;
    Vector2 dribbleLeft = RobotPos + Vector2(constants::ROBOT_RADIUS, 0).rotate(robot->angle - constants::DRIBBLER_ANGLE_OFFSET);
    Vector2 dribbleRight = RobotPos + Vector2(constants::ROBOT_RADIUS, 0).rotate(robot->angle + constants::DRIBBLER_ANGLE_OFFSET);
    if (control::ControlUtils::pointInTriangle(BallPos, RobotPos, dribbleLeft, dribbleRight)) {
        return true;
    }
        // else check the rectangle in front of the robot.
    else
        return control::ControlUtils::pointInRectangle(BallPos, dribbleLeft, dribbleRight,
                dribbleRight + Vector2(frontRange, 0).rotate(robot->angle),
                dribbleLeft + Vector2(frontRange, 0).rotate(robot->angle));
}
}
}