#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(std::move(name), std::move(blackboard)), ioManager(false, true) {
    robot = std::make_shared<roboteam_msgs::WorldRobot>();
}

void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    ioManager.publishRobotCommand(cmd);
}

std::string Skill::node_name() {
    return name;
}

Skill::Status Skill::update() {
    updateRobot();
    if (! robot) return Status::Failure;
    return onUpdate();
}

void Skill::initialize() {
    robot = getRobotFromProperties(properties);
    if (! robot) return;
    onInitialize();
}

void Skill::terminate(Status s) {
    if (! robot) return;
    onTerminate(s);
}

} // ai
} // rtt