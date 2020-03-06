#include "conditions/Condition.h"

namespace rtt::ai {

Condition::Condition(std::string name, bt::Blackboard::Ptr blackboard) : bt::Leaf(name, blackboard) {}

std::string Condition::node_name() { return name; }

void Condition::initialize() {
    robot = getRobotFromProperties(properties);
    ball = world->getBall();
    // TODO does a condition always require a robot?
    if (!robot.has_value()){
        std::cout << "[Condition::initialize] Warning. Trying to initialize Condition without the robot present" << std::endl;
        return;
    }
    // TODO does a condition always require a ball?
    if (!ball){
        std::cout << "[Condition::initialize] Warning. Trying to initialize Condition without a ball present" << std::endl;
        return;
    }
    onInitialize();
}

Condition::Status Condition::update() {
    // get a robot if the condition needs one
    if (!properties->getString("ROLE").empty()) {
        updateRobot();
        if (!robot.has_value())
            return Status::Failure;
    }

    // there should always be a ball (Emiel : Why? What if it just counts the number of robots on the field)
    ball = world->getBall();
    if (!ball.has_value())
        return Status::Waiting;

    return onUpdate();
}

void Condition::terminate(Condition::Status s) {
    // TODO check if this is correct
    if (!robot.has_value()) return;
    if (!ball) return;
    onTerminate(s);
}

}  // namespace rtt::ai