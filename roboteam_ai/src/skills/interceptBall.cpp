//
// Created by rolf on 12/12/18.
//

#include "interceptBall.h"
namespace rtt {
namespace ai {
InterceptBall::InterceptBall(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { };
std::string InterceptBall::node_name() {return "InterceptBall"; }
void InterceptBall::initialize() {
    robot=getRobotFromProperties(properties);

    keeper=properties->getBool("Keeper");

    currentProgression=INTERCEPTING;

    tickCount=0;
    maxTicks= static_cast<int>(floor(constants::MAX_INTERCEPT_TIME*constants::tickRate));
    ball=World::getBall();
    ballStartPos=ball.pos;
    ballStartVel=ball.vel;
    ballEndPos=Vector2(ball.pos)+Vector2(ball.vel)*constants::MAX_INTERCEPT_TIME;
    if(robot){interceptPos=computeInterceptPoint();}
    else currentProgression=BALLMISSED;
}
InterceptBall::Status InterceptBall::update() {
    updateRobot();
    ball=World::getBall();
    if (robot){
        checkProgression();
        switch (currentProgression){
        case INTERCEPTING: sendInterceptCommand(); break;
        case CLOSETOPOINT: sendFineInterceptCommand(); break;
        case OVERSHOOT: sendInterceptCommand(); break;
        case INPOSITION: sendStopCommand();break;
        case BALLDEFLECTED: return Status::Success;
        case BALLMISSED: return Status::Failure;
        }
        tickCount++;
    }
    else return Status::Failure;
}

void InterceptBall::checkProgression(){
    //check if we missed the ball
    if(missBall()||tickCount>maxTicks){
        currentProgression=BALLMISSED;
        return;
    }
    //Check if the ball was deflected
    if(ballDeflected()){
        currentProgression=BALLDEFLECTED;
        return;
    }

    //Update the state of the robot
    switch(currentProgression){
    case INTERCEPTING:;
    case CLOSETOPOINT:;
    case OVERSHOOT:;
    case INPOSITION:;
    case BALLDEFLECTED: return;
    case BALLMISSED: return;
    }

};
void InterceptBall::terminate(rtt::ai::Skill::Status s) {

}

Vector2 InterceptBall::computeInterceptPoint(){
    Vector2 interceptionPoint;
    if (keeper){
        Arc keeperCircle=control::ControlUtils::createKeeperArc();
        std::pair<boost::optional<Vector2>,boost::optional<Vector2>> intersections=keeperCircle.intersectionWithLine(ballStartPos,ballEndPos);
        if (intersections.first&&intersections.second){
            double dist1=(Vector2(robot->pos)-*intersections.first).length();
            double dist2=(Vector2(robot->pos)-*intersections.second).length();
            if (dist2<dist1){
                interceptionPoint=*intersections.second;
            }
            else {interceptionPoint=*intersections.first;}
        }
        else if (intersections.first){
            interceptionPoint=*intersections.first;
        }
        else if (intersections.second){
            interceptionPoint=*intersections.second;
        }
        else { // if the Line does not intercept it usually means the ball is coming from one of the corners-ish to the keeper
            interceptionPoint = Vector2(robot->pos).project(ballStartPos,
                    ballEndPos); // For now we pick the closest point to the (predicted) line of the ball
        }
    }
    else{
        interceptionPoint = Vector2(robot->pos).project(ballStartPos,
                ballEndPos); // For now we pick the closest point to the (predicted) line of the ball for any 'regular' interception
    }
    return interceptionPoint;
}
// Checks if the Robot already missed the Ball
bool InterceptBall::missBall(){

    // We check if the ball ever comes into the rectangle behind the robot that it should 'cover off'
    if ball.pos in rectangle

}
//Checks if the ball was deflected by the Robot
bool InterceptBall::ballDeflected(){

}
}
}