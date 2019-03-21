

//
// Created by rolf on 14/12/18.
//
// TODO: Test real robot rotation speeds.
// TODO: Make the robot automatically slow down/speed up if the ball is going to one end of the dribbler. Control?
#include "DribbleRotate.h"
#include "../control/ControlUtils.h"
#include "../utilities/Field.h"
#include "../coach/Ballplacement.h"

namespace rtt {
namespace ai {
DribbleRotate::DribbleRotate(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void DribbleRotate::checkProgression() {
    double angDif = Control::angleDifference(robot->angle, targetAngle);
    if (!World::ourBotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())){ // change to !botHassball() alter
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
        maxSpeed = MAX_SPEED;
    }
    if (properties->hasDouble("Angle")) {
        targetAngle = properties->getDouble("Angle");
    }
    else if (properties->getBool("RotateToTheirGoal")){
        Vector2 theirCentre=Field::get_their_goal_center();
        targetAngle=(theirCentre-robot->pos).angle();
    }
    else if (properties->getBool("BallPlacement")){
        if(properties->getBool("BallPlacementForwards")){
        }
        targetAngle=(Vector2(robot->pos) - coach::g_ballPlacement.getBallPlacementPos()).angle();
    }
    if (!properties->hasDouble("Angle")&&!properties->hasBool("RotateToTheirGoal")&&!properties->hasBool("BallPlacement")){
        ROS_ERROR(" dribbleRotate Initialize -> No good angle set in properties");
        currentProgression = FAIL;
    }
    startAngle = robot->angle;
    incrementAngle= maxSpeed/Constants::TICK_RATE();
    currentProgression=ROTATING;
    currentTick=0;
    extraTick= static_cast<int>(WAIT_TIME * Constants::TICK_RATE());
    dir=Control::rotateDirection(startAngle,targetAngle);
    maxTick=(int)floor(Control::angleDifference(startAngle,targetAngle)/maxSpeed*Constants::TICK_RATE());
    if (!World::ourBotHasBall(robot->id,Constants::MAX_BALL_RANGE())){
        std::cout<<"Robot does not have ball in dribbleRotateInitialize"<<std::endl;
        std::cout<< "Distance"<<(Vector2(robot->pos)-Vector2(ball->pos)).length() - Constants::ROBOT_RADIUS()<< "Max distance:" << Constants::MAX_BALL_RANGE() << std::endl;
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
}
}