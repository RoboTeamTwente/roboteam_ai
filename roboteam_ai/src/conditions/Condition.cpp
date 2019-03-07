#include "Condition.h"

namespace rtt {
namespace ai {

Condition::Condition(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(name, blackboard) { }

std::string Condition::node_name() {
    return name;
}

void Condition::initialize() {
    if (! properties->getString("ROLE").empty()) {
        robot = getRobotFromProperties(properties);
        ball = World::getBall(); // update ball position
        if (! robot) return;
        if (! ball) return;
    }
    onInitialize();
}

Condition::Status Condition::update() {
    if (! properties->getString("ROLE").empty()) {
        updateRobot();
        ball = World::getBall(); // update ball position
        if (! robot) return Status::Failure;
        if (! ball) return Status::Waiting;
    }
    return onUpdate();
}

void Condition::terminate(Condition::Status s) {
    if (! robot) return;
    if (! ball) return;
    onTerminate(s);
}

} // ai
} // rtt
