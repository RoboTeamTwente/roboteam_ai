//
// Created by baris on 05/11/18.
//

#include "DemoTactic.h"
#include "../../utilities/World.h"

namespace bt{

DemoTactic::DemoTactic(std::string name, Blackboard::Ptr blackboard) {

    globalBB = blackboard;
    setName(name);
}

void DemoTactic::setName(std::string newName) {
    name = newName;

}
void DemoTactic::Initialize() {

    int numberOfRobots = 1;

    auto world = rtt::ai::World::get_world();
    int ID =  world.us.begin()->id;

    RobotDealer::claimRobotForTactic(ID, "testTactic"); // TODO add a role




}
Node::Status DemoTactic::Update() {
    return Status::Invalid;
}
void DemoTactic::AddChild(bt::Node::Ptr newChild) {
    this->child = newChild;
}
}