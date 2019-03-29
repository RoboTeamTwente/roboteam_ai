//
// Created by rolf on 04/12/18.
//

#include "GetBall.h"
#include "../utilities/Constants.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

//TODO: do obstacle checking and return fail if there is an obstacle in the way.
//GetBall turns the robot to the ball and softly approaches with dribbler on in an attempt to get the ball.
GetBall::GetBall(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

// Essentially a state transition diagram. Contains much of the logic
void GetBall::checkProgression() {
    if (deltaPos.length() > MAX_RANGE ||currentTick>maxTicks) {
        currentProgress = FAIL;
        return;
    }
    double angleDif = Control::angleDifference(robot->angle, deltaPos.angle());
    if (currentProgress == TURNING) {
        if (angleDif < ANGLE_SENS) {
            currentProgress = APPROACHING;
            return;
        }
    }
    else if (currentProgress == APPROACHING) {
        if (angleDif >= ANGLE_SENS) {
            currentProgress = TURNING;
            return;
        }
        if (world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            currentProgress = OVERSHOOTING;
            return;
        }
        else{
            return;
        }
    }
    else if (currentProgress == OVERSHOOTING){
        if (!world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            currentProgress = TURNING;
            return;
        }
        if (((approachPos-robot->pos)).length()<OVERSHOOT){
            currentProgress=DRIBBLING;
            return;
        }
    }
    else if (currentProgress == DRIBBLING) {
        if (! world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            currentProgress = APPROACHING;
            count = 0;
            return;
        }
        count ++;
        if (count > POSSES_BALL_CYCLES) {
            currentProgress = SUCCESS;
            return;
        }
    }
    else if (currentProgress == FAIL || currentProgress == SUCCESS) {
        return;
    }
}

void GetBall::onInitialize() {
    currentProgress = TURNING;
    count = 0;
    double maxTime;
    if (properties->hasDouble("maxTime")){
    maxTime=properties->getDouble("maxTime");
    }
    else maxTime=1000;
    currentTick=0;
    maxTicks= static_cast<int>(floor(maxTime*Constants::TICK_RATE()));
}

GetBall::Status GetBall::onUpdate() {
    if (!ball) return Status::Running;
    deltaPos = Vector2(ball->pos) - Vector2(robot->pos);

    if(currentProgress!=OVERSHOOTING&&currentProgress!=DRIBBLING){
        approachPos= Vector2(ball->pos)+(Vector2(robot->pos)-Vector2(ball->pos)).stretchToLength(Constants::CENTRE_TO_FRONT()-0.03);
    }

    if(!world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())){
        lockedAngle=deltaPos.angle();
    }
    if (ball->visible){
        lastVisibleBallPos=ball->pos;
    }
    checkProgression();
    currentTick++;
    if (currentTick >= maxTicks) return Status::Failure;

    switch (currentProgress) {
        case TURNING:
            sendTurnCommand();
            return Status::Running;
        case APPROACHING:
            sendApproachCommand();
            return Status::Running;
        case OVERSHOOTING:
            sendOvershootCommand();
            return Status::Running;
        case DRIBBLING:
            sendDribblingCommand();
            return Status::Running;
        case SUCCESS:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }
    return Status::Failure;
}

void GetBall::onTerminate(Status s) {
    sendDribblingCommand();
}
void GetBall::sendTurnCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);
}
void GetBall::sendApproachCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.x_vel = (float) deltaPos.normalize().x * SPEED;
    command.y_vel = (float) deltaPos.normalize().y * SPEED;
    command.w = lockedAngle;
    publishRobotCommand(command);

}
void GetBall::sendOvershootCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.x_vel = (float) (approachPos-robot->pos).normalize().x * SPEED;
    command.y_vel = (float) (approachPos-robot->pos).normalize().y * SPEED;
    command.w = lockedAngle;
    publishRobotCommand(command);
}
void GetBall::sendDribblingCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = lockedAngle;
    publishRobotCommand(command);
}

bool GetBall::botHasLastVisibleBall(){
    Vector2 robotPos = robot->pos;
    double robotOrientation = robot->angle;
    Vector2 ballPos = lastVisibleBallPos;
    Vector2 dribbleLeft = robotPos
            + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation - Constants::DRIBBLER_ANGLE_OFFSET());
    Vector2 dribbleRight = robotPos
            + Vector2(Constants::ROBOT_RADIUS(), 0).rotate(robotOrientation + Constants::DRIBBLER_ANGLE_OFFSET());
    if (control::ControlUtils::pointInTriangle(ballPos, robotPos, dribbleLeft, dribbleRight)) {
        return true;
    }
    else return control::ControlUtils::pointInRectangle(ballPos, dribbleLeft, dribbleRight,
                dribbleRight + Vector2(Constants::MAX_BALL_BOUNCE_RANGE(), 0).rotate(robotOrientation),
                dribbleLeft + Vector2(Constants::MAX_BALL_BOUNCE_RANGE(), 0).rotate(robotOrientation));
}

}//rtt
}//ai