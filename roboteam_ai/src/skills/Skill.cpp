#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard) : bt::Leaf(std::move(name), std::move(blackboard)) {
    robot = std::make_shared<roboteam_msgs::WorldRobot>();
    ball = std::make_shared<roboteam_msgs::WorldBall>();

}
void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    ioManager.publishRobotCommand(cmd);
}

std::string Skill::node_name() {
    return name;
}

Skill::Status Skill::update() {
    updateRobot();
    ball = World::getBall(); // update ball position
    if (! robot) return Status::Failure;
    return onUpdate();
}

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    ball = World::getBall();
    if (!robot) return;
    onInitialize();
}

void Skill::terminate(Status s) {
    if (!robot) return;
    onTerminate(s);
}

} // ai
} // rtt