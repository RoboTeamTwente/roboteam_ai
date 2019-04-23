//
// Created by rolf on 23-4-19.
//

#include "PenaltyKeeper.h"
namespace rtt {
namespace ai {
PenaltyKeeper::PenaltyKeeper(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

void PenaltyKeeper::onInitialize() {
    goalLine = getGoalLine();
    state = WAITING;
    firstBallPos=world::world->getBall()->pos;
}

PenaltyKeeper::Status PenaltyKeeper::onUpdate() {
    state=updateState(state);
    switch (state) {
    case WAITING: {
        sendWaitCommand();
        break;
    }
    case BALLSHOT: {
        sendInterceptCommand();
        break;
    }
    case FREEMOVE: {
        if (isBallShot()){
            sendFreeMoveCommand();
        }
        else{
            sendWaitCommand();
        }
        break;
    }
    }
    return Status::Running;
}
PenaltyKeeper::PenaltyState PenaltyKeeper::updateState(PenaltyState currentState) {
    if (currentState==WAITING){
        if (freeToMove()){
            return FREEMOVE;
        }
        return isBallShot() ? BALLSHOT : WAITING;
    }
    else if (currentState==BALLSHOT){
        if (freeToMove()){
            return FREEMOVE;
        }
        isBallShot()? ballNotShotTicks++ : ballNotShotTicks=0;
        if (ballNotShotTicks>3){
            return WAITING;
        }
        return BALLSHOT;
    }
    else if (currentState==FREEMOVE){
        return FREEMOVE;
    }
    return WAITING;
}
Vector2 PenaltyKeeper::computeDefendPos() {
    auto attacker = world::world->getRobotClosestToBall(world::THEIR_ROBOTS);
    // we check the line defined by attacker's centre and the ball position
    Vector2 beginPos = attacker.pos;
    Vector2 endPos = attacker.pos
            + (world::world->getBall()->pos - attacker.pos).stretchToLength(world::field->get_field().field_length);
    //std::cout<<endPos<<std::endl;
    if (control::ControlUtils::lineSegmentsIntersect(beginPos, endPos, goalLine.first, goalLine.second)) {
        Vector2 intersect = control::ControlUtils::twoLineIntersection(beginPos, endPos, goalLine.first,
                goalLine.second);
        return intersect;
    }
    return (goalLine.first + goalLine.second)*0.5;
}

Vector2 PenaltyKeeper::interceptBallPos() {
    Vector2 startBall = world::world->getBall()->pos;
    Vector2 endBall = world::world->getBall()->pos + world::world->getBall()->vel.stretchToLength(100);
    Vector2 predictedShotLocation = control::ControlUtils::twoLineIntersection(startBall, endBall, goalLine.first,
            goalLine.second);
    double margin = 0.05;//m next to the goal
    if (predictedShotLocation.y <= world::field->get_field().goal_width*0.5 + margin
            && predictedShotLocation.y >= - world::field->get_field().goal_width*0.5 - margin) {
        return predictedShotLocation;
    }
    return (goalLine.first + goalLine.second)*0.5;
}

void PenaltyKeeper::sendWaitCommand() {
    Vector2 targetPos = computeDefendPos();
    targetPos.y=targetPos.y*0.5;
    Vector2 delta = gtp.getPosVelAngle(robot, targetPos).vel;
    command.x_vel = delta.x;
    command.y_vel = delta.y;
    command.w = 0;
    publishRobotCommand();
}
void PenaltyKeeper::sendInterceptCommand() {
    Vector2 interceptPos = interceptBallPos();
    Vector2 delta = gtp.getPosVelAngle(robot, interceptPos).vel;
    command.x_vel = delta.x;
    command.y_vel = delta.y;
    command.w = 0;
    publishRobotCommand();
}
void PenaltyKeeper::sendFreeMoveCommand() {
    Vector2 startBall = world::world->getBall()->pos;
    Vector2 endBall = world::world->getBall()->pos + world::world->getBall()->vel.stretchToLength(100);
    Vector2 interceptPos=robot->pos.project(startBall,endBall);
    if (world::field->pointIsInDefenceArea(interceptPos)){
        Vector2 delta = gtp.getPosVelAngle(robot, interceptPos).vel;
        command.x_vel=delta.x;
        command.y_vel=delta.y;
        command.w=(endBall-startBall).angle();
        publishRobotCommand();

    }
    else{
        //failsave
        sendWaitCommand();
        return;
    }
}
std::pair<Vector2, Vector2> PenaltyKeeper::getGoalLine() {
    std::pair<Vector2, Vector2> originalLine = world::field->getGoalSides(true);
    double forwardX = originalLine.first.x + Constants::KEEPER_PENALTY_LINE_MARGIN();
    originalLine.first.x = forwardX;
    originalLine.second.x = forwardX;
    return originalLine;
}
bool PenaltyKeeper::isBallShot() {
    return world::world->getBall()->vel.x>-0.4;
}
bool PenaltyKeeper::freeToMove() {
    return (firstBallPos-world::world->getBall()->pos).length()>0.05;
}
}
}
