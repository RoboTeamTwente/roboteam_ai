#include "Condition.h"

namespace rtt {
namespace ai {

Condition::Condition(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(name, blackboard) { }

std::string Condition::node_name() {
    return name;
}

void Condition::initialize() {
    ball = world::world->getBall(); // update ball position
    if (! properties->getString("ROLE").empty()) {
        robot = getRobotFromProperties(properties);
        if (! robot) return;
    }
    if (! ball) return;

    onInitialize();
}

Condition::Status Condition::update() {

    // get a robot if the condition needs one
    if (! properties->getString("ROLE").empty()) {
        updateRobot();
        if (! robot) return Status::Failure;
    }

    // there should always be a ball
    ball = world::world->getBall(); // update ball position
    if (! ball) return Status::Waiting;

    return onUpdate();
}

void Condition::terminate(Condition::Status s) {
    if (! robot) return;
    if (! ball) return;
    onTerminate(s);
}

} // ai
} // rtt
