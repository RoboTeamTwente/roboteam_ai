//
// Created by rolf on 30-1-19.
//

#include "GoAroundPos.h"
namespace rtt {
namespace ai {
GoAroundPos::GoAroundPos(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
void GoAroundPos::onInitialize() {
    if (properties->hasVector2("targetPos")) {
        ballIsTarget = false;
        targetPos = properties->getVector2("targetPos");
    }
    else {
        ballIsTarget = true;
        if (ball) {
            targetPos = ball->pos;
        }
        else {
            targetPos = {0, 0};
            ROS_ERROR_STREAM("GoAroundPos update--> No ball found! Defaulting to {0,0}");
        }
    }
    if (properties->hasDouble("targetDir")) {
        endAngle = Control::constrainAngle(properties->getDouble("targetDir"));
    }
    else if (properties->getBool("towardsTheirGoal")) {
        endAngle = (Field::get_their_goal_center() - targetPos).angle();
    }
    else {
        endAngle = 0;
        ROS_ERROR_STREAM("GoAroundPos update --> No target direction set! Defaulting to 0");
    }

    deltaPos = targetPos - robot->pos;
    startAngle = deltaPos.angle();
    rotateDir = Control::rotateDirection(startAngle, endAngle);
    distanceFromPoint = 0.15;
    distanceError = 0.05;
    currentTick = 0;
    rotatingSpeed = 1.0;
    angleDif = Control::angleDifference(startAngle, endAngle);
    maxTick = floor(angleDif/rotatingSpeed)*constants::tickRate;
    currentProgress=ROTATING;
}
GoAroundPos::Status GoAroundPos::onUpdate() {
    if (! robot) { return Status::Failure; }
    if (ballIsTarget && ! ball) {ROS_ERROR_STREAM("GoAroundPos update -> No ball found!");return Status::Failure; }
    if (ballIsTarget) {
        targetPos = ball->pos;
    }
    if (currentTick <= maxTick) {
        commandPos =
                targetPos + Vector2(distanceFromPoint, 0).rotate(startAngle + rotateDir*currentTick/maxTick*angleDif);
    }
    deltaPos = targetPos - robot->pos;
    currentProgress=checkProgression();
    currentTick++;
    switch(currentProgress){
    case ROTATING: sendRotateCommand(); return Status::Running;
    case STOPPING: sendRotateCommand(); return Status::Running;
    case FAIL: return Status::Failure;
    case DONE: return Status::Success;
    }

}
void GoAroundPos::onTerminate(rtt::ai::Skill::Status s) {
    roboteam_msgs::RobotCommand command;
    Vector2 deltaCommandPos = (commandPos - robot->pos).normalize();
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);

}
GoAroundPos::Progression GoAroundPos::checkProgression() {
    //Failure condition: If it goes outside of the margin during any phase but stopping (also helps against ball moving etc.)
    if (currentProgress!=STOPPING){
        if (!checkPosition()){
            return FAIL;
        }
    }
    //Go to stopping if we are done rotating
    if (currentProgress==ROTATING){
        if (currentTick>maxTick){
            return STOPPING;
        }
        else return ROTATING;
    }
    if (currentProgress==STOPPING){
        //Done when robot sufficiently close to desired end position. There might be more pretty solutions
        if ((commandPos-robot->pos).length()<0.03){
            return  DONE;
        }
        //If Robot takes too long to stop, fail
        if (currentTick>maxTick+constants::MAX_GOAROUND_STOP_TIME*constants::tickRate){
            return FAIL;
        }
    }
}
bool GoAroundPos::checkPosition() {
    double currentAngle=deltaPos.angle();
    if (Control::angleDifference(startAngle,currentAngle)+Control::angleDifference(currentAngle,endAngle)>angleDif){
        return false;
    }
    return ((deltaPos.length()<=(distanceFromPoint+distanceError))&&deltaPos.length()>distanceFromPoint-distanceError);
}
void GoAroundPos::sendRotateCommand() {
    roboteam_msgs::RobotCommand command;
    Vector2 deltaCommandPos = (commandPos - robot->pos).normalize();
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 0;
    command.x_vel = deltaCommandPos.x*1.0;
    command.y_vel = deltaCommandPos.y*1.0;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);
}
}
}