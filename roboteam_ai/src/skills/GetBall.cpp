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
    if (deltaPos.length() > Constants::MAX_GETBALL_RANGE() ||currentTick>maxTicks) {
        currentProgress = FAIL;
        //std::cout<<"GetBall-> FAIL";
        return;
    }
    double angleDif = Control::angleDifference(robot->angle, deltaPos.angle());
    if (currentProgress == TURNING) {
        if (angleDif < Constants::ANGLE_SENS()) {
            currentProgress = APPROACHING;
            //std::cout<<"GetBall: TURNING->APPROACHING"<<std::endl;
            return;
        }
    }
    else if (currentProgress == APPROACHING) {
        if (angleDif >= Constants::ANGLE_SENS()) {
            currentProgress = TURNING;
            //std::cout<<"GetBall: APPROACHING-> TURNING"<<std::endl;
            return;
        }
        if (world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            //std::cout<<"GetBall: APPROACHING -> OVERSHOOTING"<<std::endl;
            currentProgress = OVERSHOOTING;
            return;
        }
        else{
            return;
        }
    }
    else if (currentProgress == OVERSHOOTING){
        if (!world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            //std::cout<<"GetBall: OVERSHOOTING -> TURNING"<<std::endl;
            currentProgress = TURNING;
            return;
        }
        if (((approachPos-robot->pos)).length()<Constants::GET_BALL_OVERSHOOT()){
            //std::cout<<"GetBall: OVERSHOOTING -> DRIBBLING"<<std::endl;
            currentProgress=DRIBBLING;
            return;
        }
//        if (World::getBall()->visible){
//            if(World::ourBotHasBall(robot->id,Constants::MAX_BALL_RANGE())){
//                currentProgress=DRIBBLING;
//                std::cout<<"GetBall: OVERSHOOTING -> DRIBBLING"<<std::endl;
//                return;
//            }
//        }
//        else{
//            // no visible ball: we check relative to last ballPos
//            if(botHasLastVisibleBall()){
//                currentProgress=DRIBBLING;
//                std::cout<<"GetBall: OVERSHOOTING -> DRIBBLING"<<std::endl;
//                return;
//            }
//        }
    }
    else if (currentProgress == DRIBBLING) {
        if (! world::world->ourRobotHasBall(robot->id,Constants::MAX_BALL_BOUNCE_RANGE())) {
            currentProgress = APPROACHING;
            count = 0;
            //std::cout<<"GetBall: DRIBBLING-> APPROACHING"<<std::endl;
            return;
        }
        count ++;
        if (count > Constants::POSSES_BALL_CYCLES()) {
            currentProgress = SUCCESS;
            //std::cout<<"GetBall: SUCCESS"<<std::endl;
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

    if (currentProgress == TURNING) {
        sendTurnCommand();
    }
    else if (currentProgress == APPROACHING) {
        sendApproachCommand();
    }
    else if (currentProgress == OVERSHOOTING){
        sendOvershootCommand();
    }
    else if (currentProgress == DRIBBLING) {
        sendDribblingCommand();
    }
    switch (currentProgress) {
        case TURNING:
            return Status::Running;
        case APPROACHING:
            return Status::Running;
        case OVERSHOOTING:
            return Status::Running;
        case DRIBBLING:
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
    command.x_vel = (float) deltaPos.normalize().x * Constants::GETBALL_SPEED();
    command.y_vel = (float) deltaPos.normalize().y * Constants::GETBALL_SPEED();
    command.w = lockedAngle;
    publishRobotCommand(command);

}
void GetBall::sendOvershootCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.x_vel = (float) (approachPos-robot->pos).normalize().x * Constants::GETBALL_SPEED();
    command.y_vel = (float) (approachPos-robot->pos).normalize().y * Constants::GETBALL_SPEED();
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