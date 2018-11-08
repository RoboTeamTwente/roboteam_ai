#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(name, blackboard), ioManager(false, true) {
}

void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
    ioManager.publishRobotCommand(cmd);
}

} // ai
} // rtt