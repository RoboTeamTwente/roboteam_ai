#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(name, blackboard), ioManager(false, true) {
    robot = std::make_shared<roboteam_msgs::WorldRobot>();
}

void Skill::terminate(Status s) { }

void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    ioManager.publishRobotCommand(cmd);
}
std::string Skill::node_name() {
    return name;
}

} // ai
} // rtt