//
// Created by rolf on 11-4-19.
//

#include "SlingShot.h"

namespace rtt {
namespace ai {

SlingShot::SlingShot(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
void SlingShot::onInitialize() {
    waitingTicks=0;
    dribbledTicks=0;
    ballShotTicks=0;
    if (! world::world->robotHasBall(robot->id, true)) {
        progression = FAIL;
        return;
    }
    progression = DRIBBLING;
    rotateAngle=robot->angle;
    kickOrient=robot->angle;
}
Skill::Status SlingShot::onUpdate() {
    progression=updateProgress(progression);

    switch (progression){
    case DRIBBLING: sendDribbleCommand(); return Status::Running;
    case ROTATINGAWAY: sendRotateCommand(); return Status::Running;
    case WAITINGFORRESULT: sendWaitCommand();return Status::Running;
    case FAIL: sendWaitCommand(); return Status::Failure;
    case SUCCESS: sendWaitCommand();return Status::Success;
    }
    return Status::Failure;
}
void SlingShot::onTerminate(Skill::Status status) {

}
SlingShot::Progression SlingShot::updateProgress(Progression currentProgress) {
    int maxDribbleTicks = 40;
    int maxWaitingTicks= 20;
    // we go from dribbling to rotating away to waiting for a result.
    if (currentProgress == DRIBBLING) {
        if (dribbledTicks < maxDribbleTicks) {
            dribbledTicks ++;
            return world::world->robotHasBall(robot->id, true) ? DRIBBLING : FAIL;
        }
        else {
            kickOrient=robot->angle;
            rotateAngle=control::ControlUtils::constrainAngle(robot->angle+M_PI);
            return ROTATINGAWAY;
        }
    }
    // if the robot is at the angle we start waiting
    else if (currentProgress == ROTATINGAWAY) {
        if (robotAtAngle()) {
            return WAITINGFORRESULT;
        }
    }
    else if (currentProgress==WAITINGFORRESULT) {
        waitingTicks++;
        // if we detect the ball being shot return success
        if (ballShot()){
            ballShotTicks++;
        }
        if (ballShotTicks>3){
            return SUCCESS;
        }
        else if (waitingTicks>maxWaitingTicks){
            return FAIL;
        }
    }
    return FAIL;
}
bool SlingShot::ballShot() {
    Vector2 vectorFromStart=kickPos-ball->pos;
    double vectorFromStartAngle=vectorFromStart.angle();
    Angle diff=Angle(kickOrient-vectorFromStartAngle);
    // check if the ball has gone to direction we expect for more than 2 centimeters
    return abs(diff.getAngle()) > M_PI_2 && vectorFromStart.length() > 0.02;

}
bool SlingShot::robotAtAngle() {
    double margin=0.05*M_PI;
    return robot->angle.angleDiff(rotateAngle) < margin;
}
void SlingShot::sendDribbleCommand() {
    command.dribbler=true; //TODO:check if we can control velocities
    command.x_vel=0;
    command.y_vel=0;
    command.w=robot->angle;
    publishRobotCommand();
}
void SlingShot::sendRotateCommand() {
    command.dribbler=false;
    command.x_vel=0;
    command.y_vel=0;
    command.w=rotateAngle;
    publishRobotCommand();
}
void SlingShot::sendWaitCommand() {
    command.dribbler=false;
    command.x_vel=0;
    command.y_vel=0;
    command.w=rotateAngle;
    publishRobotCommand();
}
}//ai
}//rtt