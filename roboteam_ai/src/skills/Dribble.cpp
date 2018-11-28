//
// Created by rolf on 28/11/18.
//

#include "Dribble.h"

namespace rtt{
namespace ai{
Dribble::Dribble(string name, bt::Blackboard::Ptr blackboard) :Skill(name, blackboard){}

Dribble::Progression Dribble::checkProgression() { }

bool Dribble::robotHasBall(){
    //The ball is in an area defined by a cone from the robot centre, or from a rectangle in front of the dribbler
    Vector2 posdif=robot.pos-ball.pos;
    if (posdif.length()<rtt::ai::constants
    }
    else if {

    }
    else
        return false;
}
void Dribble::initialize() {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("Dribble Initialize -> robot does not exist in world");
            currentProgress = Progression::FAIL;
            return;
        }
    }
    else {
        ROS_ERROR("Dribble Initialize -> ROLE INVALID!!");
        currentProgress = Progression::FAIL;
        return;
    }
    //TODO: add failchecking if ball does not exist.
    ball=World::getBall();
//  ____________________________________________________

    forwardDirection=properties->getBool("forwardDirection");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("Dribble Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
    }

    if(!Dribble::robotHasBall()){
        ROS_ERROR("Dribble Initialize -> Robot does not have the ball!");
        currentProgress = Progression::FAIL;
    }

}

Dribble::status Dribble::update() {

}
void Dribble::terminate(Dribble::status s) {;}
std::string Dribble::node_name() {return "Dribble";}

}
}
