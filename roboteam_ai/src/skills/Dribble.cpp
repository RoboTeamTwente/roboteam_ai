//
// Created by rolf on 28/11/18.
//

#include "Dribble.h"
namespace rtt {
namespace ai {

Dribble::Dribble(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

namespace c=rtt::ai::constants;
Dribble::Progression Dribble::checkProgression() {
    if (currentProgress == ON_THE_WAY) {
        if (! robotHasBall()) {
            return FAIL;
        }
        if (deltaPos.length() <= constants::DRIBBLE_POSDIF) {
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
        if (! robotHasBall()) {
            return FAIL;
        }
        else if (count >= maxTicks) {
            return DONE;
        }
        else return STOPPED;
    }
    else if (currentProgress == DONE) {
        return DONE;
    }
    else if (currentProgress == FAIL) {
        return FAIL;
    }
    else if (currentProgress == WAITING) {
        return WAITING;
    }

    return FAIL;
}

bool Dribble::robotHasBall() {
    //The ball is in an area defined by a cone from the robot centre, or from a rectangle in front of the dribbler
    if(!ball->visible){
        return true;
    }
    Vector2 RobotPos = robot->pos;
    Vector2 BallPos = ball->pos;
    return World::robotHasBall(*robot, *ball);
}
void Dribble::onInitialize() {
    //if false, robot will dribble to the position backwards with the ball.
    forwardDirection = properties->getBool("forwardDirection");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else if (properties->getBool("BallPlacement")){
        targetPos=Coach::getBallPlacementPos();
    }
    else {
        ROS_ERROR("Dribble Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
        return;
    }
    if (properties->hasInt("maxTicks")) {
        maxTicks = properties->getInt("maxTicks");
    }
    else {
        ROS_ERROR("Dribble Initialize -> No maxTicks set!");
    }
    if (! Dribble::robotHasBall()) {
        ROS_ERROR("Dribble Initialize -> Robot does not have the ball!");
        currentProgress = Progression::FAIL;
        return;
    }
    if(!ball->visible){
        auto world=World::get_world();
        Vector2 ballPos=Vector2(robot->pos)+Vector2(constants::ROBOT_RADIUS+constants::BALL_RADIUS,0).rotate(robot->angle);
        world.ball.visible=true;
        world.ball.pos=ballPos;
        World::set_world(world);
    }
    currentProgress = Progression::ON_THE_WAY;
    count = 0;

    stoppingAngle = robot->angle; // default to the current angle
    initialAngle=robot->angle;
}

Dribble::Status Dribble::onUpdate() {
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }
    else if (currentProgress == Progression::WAITING) {
        return Status::Waiting;
    }
    if(ball->visible) {
        deltaPos = targetPos - Vector2(ball->pos);
    }
    else{
        deltaPos=targetPos-(Vector2(robot->pos)+Vector2(constants::ROBOT_RADIUS+constants::BALL_RADIUS,0).rotate(robot->angle));
    }
    currentProgress = checkProgression();

    if (currentProgress == STOPPED) {
        sendStopCommand();
    }
    else if (currentProgress == ON_THE_WAY) {
        sendMoveCommand();
    }

    switch (currentProgress) {
        case ON_THE_WAY: return Status::Running;
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
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand(command);
}

void Dribble::sendMoveCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    if (forwardDirection) {
        command.w = initialAngle+M_PI;
    }
    else {
        command.w = (float) Control::constrainAngle(initialAngle);
    }
    std::vector<Vector2> dposvec = {deltaPos};
    command.dribbler = 1;
//    if(deltaPos.length()>0.5){
//    command.x_vel = (float) deltaPos.normalize().x*c::DRIBBLE_SPEED;
//    command.y_vel = (float) deltaPos.normalize().y*c::DRIBBLE_SPEED;
//    }
//    else{
        command.x_vel=(float) deltaPos.normalize().x*c::DRIBBLE_SPEED;
        command.y_vel=(float) deltaPos.normalize().y*c::DRIBBLE_SPEED;
//    }
    publishRobotCommand(command);
}
void Dribble::sendStopCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;    command.w = stoppingAngle;

    command.use_angle = 1;
    command.w = stoppingAngle;
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand(command);
}

} // ai
} // rtt
