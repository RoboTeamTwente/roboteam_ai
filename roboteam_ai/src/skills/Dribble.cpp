//
// Created by rolf on 28/11/18.
//

#include <roboteam_ai/src/coach/Ballplacement.h>
#include "Dribble.h"
namespace rtt {
namespace ai {

Dribble::Dribble(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

Dribble::Progression Dribble::checkProgression() {
    if (currentProgress == ON_THE_WAY) {
        if (! world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            return FAIL;
        }
        if (deltaPos.length() <= POS_DIF) {
            if (forwardDirection) {
                stoppingAngle = (float) deltaPos.angle();
            }
            else {
                stoppingAngle = (float) deltaPos.rotate(M_PI).angle();
            }
            return STOPPED;
        }
        else { return ON_THE_WAY; }
    }
    else if (currentProgress == STOPPED) {
        count ++;
        //ROS_WARN_STREAM("Stopped ticks #:" << count<<"/"<<maxTicks);
        if (! world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
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
    else if (properties->getBool("BallPlacement")){
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

    if (!world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
        ROS_ERROR("Dribble Initialize -> Robot does not have the ball!");
        currentProgress = Progression::FAIL;
        return;
    }
    currentProgress = Progression::ON_THE_WAY;
    count = 0;

    stoppingAngle = robot->angle; // default to the current angle
    initialAngle = robot->angle;
}

Dribble::Status Dribble::onUpdate() {
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }
    else if (currentProgress == Progression::WAITING) {
        return Status::Waiting;
    }
    deltaPos = targetPos - Vector2(ball->pos);
    currentProgress = checkProgression();

    if (currentProgress == STOPPED) {
        sendStopCommand();
    }
    else if (currentProgress == ON_THE_WAY) {
        sendMoveCommand();
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
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    if (forwardDirection) {
        command.w = (float) stoppingAngle;
    }
    else {
        command.w = (float) stoppingAngle;
    }
    if (properties->getBool("dribbleOnTerminate")){
        command.dribbler=1;
    } else{
        command.dribbler = 0;
    }
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand(command);
}

void Dribble::sendMoveCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    if (forwardDirection) {
        command.w = (float) Control::constrainAngle(initialAngle+M_PI);
    }
    else {
        command.w = (float) Control::constrainAngle(initialAngle);
    }
    std::vector<Vector2> dposvec = {deltaPos};
    command.dribbler = 1;
    command.x_vel= static_cast<float>(deltaPos.normalize().x * SPEED);
    command.y_vel= static_cast<float>(deltaPos.normalize().y * SPEED);
    publishRobotCommand(command);
}
void Dribble::sendStopCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.w = stoppingAngle;
    command.use_angle = 1;
    command.w = stoppingAngle;
    if (properties->getBool("dribbleOnTerminate")){
        command.dribbler=1;
    } else{
        command.dribbler = 0;
    }
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand(command);
}

} // ai
} // rtt
