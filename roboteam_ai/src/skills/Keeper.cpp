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

}
Keeper::Status Keeper::update() {
    updateRobot();
    if (robot){

    }
}

void Keeper::terminate(Status s){
}
}
}