//
// Created by rolf on 10/12/18.
//

#include "Keeper.h"
namespace rtt {
namespace ai {
Keeper::Keeper(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }
std::string Keeper::node_name() { return "Keeper"; }

void Keeper::initialize() {
    robot=getRobotFromProperties(properties);

    goalPos=Field::get_our_goal_center();
    goalwidth=Field::get_field().goal_width;

    //Create arc for keeper to drive on
    double diff=constants::KEEPER_POST_MARGIN-constants::KEEPER_CENTREGOAL_MARGIN;
    double radius=diff*0.5+goalwidth*goalwidth/(8*diff);
    double angle=asin(goalwidth/2/radius);
    Vector2 center=Vector2(goalPos.x+constants::KEEPER_CENTREGOAL_MARGIN+radius,0);
    blockCircle=Arc(center,radius,angle-M_PI,M_PI-angle);

}
Keeper::Status Keeper::update() {
    updateRobot();
    Vector2 ballPos=World::getBall().pos;
    Vector2 blockPoint=computeBlockPoint(ballPos);
    if (robot){
    }
    else{
        return Status::Failure;
    }
}

void Keeper::terminate(Status s){

}

Vector2 Keeper::computeBlockPoint(Vector2 defendPos) {
    Vector2 u1=(goalPos+Vector2(0.0,goalwidth*0.5)-defendPos).normalize();
    Vector2 u2=(goalPos+Vector2(0.0,-goalwidth*0.5)-defendPos).normalize();
    double dist=(defendPos-goalPos).length();
    Vector2 blockLineStart=defendPos + (u1+u2).stretchToLength(dist);
    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections =blockCircle.intersectionWithLine(blockLineStart,defendPos);
    Vector2 blockPos;

    // go stand on the intersection.
    if(intersections.first){
        blockPos=*intersections.first;
    }
    else if (intersections.second){
        blockPos=*intersections.second;
    }
    else {
        blockPos=Vector2(goalPos.x+constants::KEEPER_POST_MARGIN,goalwidth/2*signum(defendPos.y)); // Go stand at one of the poles depending on the side the defendPos is on.
    }
    return blockPos;
}
}
}