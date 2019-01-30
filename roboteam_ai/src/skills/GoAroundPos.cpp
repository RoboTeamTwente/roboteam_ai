//
// Created by rolf on 30-1-19.
//

#include "GoAroundPos.h"
#include "../interface/drawer.h"
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
        endAngle = Control::constrainAngle((Field::get_their_goal_center() - targetPos).angle());
    }
    else {
        endAngle = 0;
        ROS_ERROR_STREAM("GoAroundPos update --> No target direction set! Defaulting to 0");
    }

    deltaPos = targetPos - robot->pos;
    startAngle = deltaPos.angle();
    rotateDir = Control::rotateDirection(startAngle, endAngle);
    distanceFromPoint = 0.11;
    distanceError = 0.5;
    currentTick = 0;
    rotatingSpeed = 6.0;
    angleDif = Control::angleDifference(startAngle, endAngle);
    maxTick = floor(angleDif/rotatingSpeed*constants::tickRate);
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
                targetPos + Vector2(distanceFromPoint, 0).rotate(startAngle + rotateDir*currentTick/maxTick*angleDif+M_PI);
    }
    deltaPos = targetPos - robot->pos;
    currentProgress=checkProgression();
    std::cout<<currentProgress<<std::endl;
    currentTick++;
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    displayColorData.emplace_back(std::make_pair(commandPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(targetPos + Vector2(distanceFromPoint, 0).rotate(endAngle+M_PI),Qt::red));
    interface::Drawer::setGoToPosLuThPoints(robot->id,displayColorData);
    switch(currentProgress){
    case ROTATING: sendRotateCommand(); return Status::Running;
    case STOPPING: sendRotateCommand(); return Status::Running;
    case FAIL: std::cout<<"Failure!!"<<std::endl; return Status::Failure;
    case DONE: std::cout<<"Success!!"<<std::endl; return Status::Success;
    }

}
void GoAroundPos::onTerminate(rtt::ai::Skill::Status s) {
    roboteam_msgs::RobotCommand command;
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
            checkPosition();
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
        //Done when robot sufficiently close to desired end position and rotation.
        double angDif=Control::angleDifference(deltaPos.angle(),endAngle);
        double posDif=(commandPos-robot->pos).length();
        if (posDif<0.1&&angDif<0.03*M_PI){
            return  DONE;
        }
        //If Robot takes too long to stop, fail
        if (currentTick>maxTick+constants::MAX_GOAROUND_STOP_TIME*constants::tickRate){
            double test= deltaPos.angle();
            return FAIL;
        }
        else return STOPPING;
    }
}
bool GoAroundPos::checkPosition() {
    double currentAngle=deltaPos.angle();
    double totalSum=Control::angleDifference(startAngle,currentAngle)+Control::angleDifference(currentAngle,endAngle);
    double margin=0.05*M_PI;
    if (totalSum>angleDif+margin*2){
        return false;
    }
    return ((deltaPos.length()<=(distanceFromPoint+distanceError))&&deltaPos.length()>distanceFromPoint-distanceError);
}
void GoAroundPos::sendRotateCommand() {
    roboteam_msgs::RobotCommand command;
    Vector2 deltaCommandPos = (commandPos - robot->pos);
    deltaCommandPos=Control::VelocityLimiter(deltaCommandPos,distanceFromPoint*rotatingSpeed,0.25);
    std::cout<<deltaCommandPos.length()<<std::endl;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 0;
    command.x_vel = deltaCommandPos.x;
    command.y_vel = deltaCommandPos.y;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);
}
}
}