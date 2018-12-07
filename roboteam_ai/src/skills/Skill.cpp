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

std::shared_ptr<roboteam_msgs::WorldRobot> Skill::getRobotFromProperties(bt::Blackboard::Ptr properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robotId, true)) {
            robot = World::getRobotForId(robotId, true);
        } else {
            ROS_ERROR("%s Initialize -> robot does not exist in world", node_name().c_str());
        }
    } else {
        ROS_ERROR("%s Initialize -> ROLE WAITING!!", node_name().c_str());
    }
    return nullptr;
}

void Skill::updateRobot() {
    if (World::getRobotForId(robotId, true)) {
        robot = World::getRobotForId(robotId, true);
    } else {
        ROS_ERROR("%s Update -> robot does not exist in world", node_name().c_str());
    }
}

} // ai
} // rtt