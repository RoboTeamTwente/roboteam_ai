//
// Created by rolf on 28/11/18.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "Dribble.h"
namespace rtt {
namespace ai {

Dribble::Dribble(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

Dribble::Progression Dribble::checkProgression() {
    if (currentProgress == ON_THE_WAY) {
        if (! robot->hasBall()) {
            return FAIL;
        }
        if (deltaPos.length() < POS_DIF) {
            if (forwardDirection) {
                stoppingAngle = (float) deltaPos.angle();
            } else {
                stoppingAngle = (float) deltaPos.rotate(M_PI).angle();
            }
            return STOPPED;
        }
        else { return ON_THE_WAY; }
    }
    else if (currentProgress == STOPPED) {
        count ++;
        //ROS_WARN_STREAM("Stopped ticks #:" << count<<"/"<<maxTicks);
        if (! robot->hasBall()) {
            return FAIL;
        }
        else if (count >= maxTicks) {
            return DONE;
        } else {
            return STOPPED;
        }
    }
    else return currentProgress;
}

void Dribble::onInitialize() {
    //if false, robot will dribble to the position backwards with the ball.
    forwardDirection = properties->getBool("forwardDirection");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else if (properties->getBool("ballPlacement")){
        targetPos=coach::g_ballPlacement.getBallPlacementPos();
    }

    if (properties->hasInt("maxTicks")) {
        maxTicks = properties->getInt("maxTicks");
    }
    else {
        ROS_ERROR("Dribble Initialize -> No maxTicks set!");
    }

    if (properties->hasDouble("distance")) {
        distance = properties->getDouble("distance");
        double targetAngle;
        if (forwardDirection) {
            targetAngle = robot->angle;
        } else {
            targetAngle = control::ControlUtils::constrainAngle(robot->angle - M_PI);
        }
        targetPos = (Vector2)robot->pos + Vector2({distance, 0}).rotate(targetAngle);
    }

    if (!robot->hasBall()) {
        ROS_ERROR("Dribble Initialize -> RobotPtr does not have the ball!");
        currentProgress = Progression::FAIL;
        return;
    }
    currentProgress = Progression::ON_THE_WAY;
    count = 0;

    stoppingAngle = robot->angle; // default to the current angle
    initialAngle = robot->angle;
}

Dribble::Status Dribble::onUpdate() {
    currentProgress = checkProgression();

    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }
    else if (currentProgress == Progression::WAITING) {
        return Status::Waiting;
    }
    deltaPos = targetPos - Vector2(ball->pos);

    if (currentProgress == STOPPED) {
        sendStopCommand();
    }
    else if (currentProgress == ON_THE_WAY) {
        auto c = ballHandlePosControl.getRobotCommand(robot, targetPos, robot->angle);
        command = c.makeROSCommand();
        publishRobotCommand();
        //sendMoveCommand();
    }

    switch (currentProgress) {
        case ON_THE_WAY: return Status::Running;
        case STOPPING: return Status::Running;
        case STOPPED: return Status::Running;
        case DONE: return Status::Success;
        case FAIL: return Status::Failure;
        default: return Status::Waiting;
    }
}

void Dribble::onTerminate(Status s) {
    if (forwardDirection) {
        command.w = (float) stoppingAngle;
    }
    else {
        command.w = (float) stoppingAngle;
    }
    if (properties->getBool("dribbleOnTerminate")){
        command.dribbler=20;
    } else{
        command.dribbler = 0;
    }
    command.x_vel = 0;
    command.y_vel = 0;

    currentProgress=ON_THE_WAY;
    count=0;
    stoppingAngle = robot->angle; // default to the current angle
    initialAngle = robot->angle;

    publishRobotCommand();
}

void Dribble::sendMoveCommand() {
    if (forwardDirection) {
        command.w = (float) Control::constrainAngle(initialAngle+M_PI);
    }
    else {
        command.w = (float) Control::constrainAngle(initialAngle);
    }
    std::vector<Vector2> dposvec = {deltaPos};
    command.dribbler = 20;
    command.x_vel= static_cast<float>(deltaPos.normalize().x * SPEED);
    command.y_vel= static_cast<float>(deltaPos.normalize().y * SPEED);
    publishRobotCommand();
}

void Dribble::sendStopCommand() {
    command.w = stoppingAngle;
    if (properties->getBool("dribbleOnTerminate")){
        command.dribbler=20;
    } else{
        command.dribbler = 0;
    }
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand();
}

} // ai
} // rtt
