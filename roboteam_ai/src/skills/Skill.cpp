#include "Skill.h"
#include "../io/StrategyIOManager.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
    : bt::Leaf(name, blackboard) {
}

void Skill::publishRobotCommand(roboteam_msgs::RobotCommand cmd) {
  roleIOManager.publishRobotCommand(cmd);
}

} // ai
} // rtt