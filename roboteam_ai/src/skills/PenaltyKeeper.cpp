//
// Created by rolf on 23-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Ball.h>
#include <roboteam_ai/src/world/Robot.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/interface/api/Output.h>
#include "PenaltyKeeper.h"

namespace rtt {
namespace ai {
PenaltyKeeper::PenaltyKeeper(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

void PenaltyKeeper::onInitialize() {
    goalLine = getGoalLine();
    state = WAITING;
    firstBallPos=world::world->getBall()->pos;
    preparation=properties->getBool("prepare");
    gtp.setAutoListenToInterface(false);
}

PenaltyKeeper::Status PenaltyKeeper::onUpdate() {

    state=updateState(state);
    if (preparation){
        state=WAITING;
    }
    switch (state) {
    case WAITING: {
        sendWaitCommand();
        break;
    }
    case BALLSHOT: {
        sendInterceptCommand();
        break;
    }
    }
    return Status::Running;
}
PenaltyKeeper::PenaltyState PenaltyKeeper::updateState(PenaltyState currentState) {
    if (currentState==WAITING){
        //ballShotTicks=0;
        if (isBallShot()){
            /*
            initialPos=robot->pos;
            initialVel=robot->vel;
            */
            return BALLSHOT;
        }
        return WAITING;
    }
    else if (currentState==BALLSHOT) {
        //ballShotTicks++;
        //prints for testing: easy to measure delay/effectiveness of our strategy
//        std::cout<<"BallPtr speed: "<<world::world->getBall()->vel<<std::endl;
//        std::cout<<"Pos diff(m) :" << (robot->pos-initialPos).length() << " Vel diff : " << (robot->vel-initialVel).length() <<" tick: "<<ballShotTicks<<std::endl;
        if (isBallShot()){
            ballNotShotTicks = 0;
        }
        else{
            ballNotShotTicks ++;
        }
        if (ballNotShotTicks > 3) {
            return WAITING;
        }
        return BALLSHOT;
    }
    return WAITING;
}
Vector2 PenaltyKeeper::computeDefendPos() {
    auto attacker = world::world->getRobotClosestToBall(THEIR_ROBOTS);
    // we check the line defined by attacker's centre and the ball position
    Vector2 middle=(goalLine.first + goalLine.second)*0.5;

    if (attacker) {
        Vector2 beginPos = attacker->pos;
        Vector2 endPos = attacker->pos
                + (world::world->getBall()->pos - attacker->pos).stretchToLength(
                        world::field->get_field().field_length);

        double maxMoveDist=(world::field->get_field().goal_width-Constants::ROBOT_RADIUS())/2-0.2; // we estimate we can move the robot about 20 cm during the shot and the opponent cannot shoot perfectly within 5 cm.
        LineSegment shootLine(beginPos,endPos);
        Line goalKeepingLine(goalLine.first,goalLine.second);
        auto intersection=goalKeepingLine.intersects(shootLine);
        if (intersection) {
            if (intersection->y>maxMoveDist){
                return Vector2(middle.x,maxMoveDist);
            }
            else if(intersection->y<-maxMoveDist){
                return Vector2(middle.x,-maxMoveDist);
            }
            return *intersection;
        }
    }
    return middle;
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
    gtp.setAutoListenToInterface(false);

    gtp.updatePid(interface::Output::getKeeperPid());

    Vector2 targetPos = computeDefendPos();

    Vector2 delta = gtp.getRobotCommand(robot, targetPos).vel;
    command.x_vel = delta.x;
    command.y_vel = delta.y;
    command.w = M_PI_2;
    publishRobotCommand();
}
void PenaltyKeeper::sendInterceptCommand() {
    gtp.setAutoListenToInterface(false);
    gtp.updatePid({5.2,0.0,0.2});

    Vector2 interceptPos = interceptBallPos();
    Vector2 delta = gtp.getRobotCommand(robot, interceptPos).vel;
    command.x_vel = delta.x;
    command.y_vel = delta.y;
    command.w = M_PI_2;
    publishRobotCommand();
}
std::pair<Vector2, Vector2> PenaltyKeeper::getGoalLine() {
    std::pair<Vector2, Vector2> originalLine = world::field->getGoalSides(true);
    double forwardX = originalLine.first.x + Constants::KEEPER_PENALTY_LINE_MARGIN();
    originalLine.first.x = forwardX;
    originalLine.second.x = forwardX;
    return originalLine;
}
bool PenaltyKeeper::isBallShot() {
    return world::world->getBall()->vel.x<-0.2;
}
void PenaltyKeeper::onTerminate(rtt::ai::Skill::Status s) {
    state=WAITING;
    ballNotShotTicks=0;
    goalLine=getGoalLine();
}
}
}
