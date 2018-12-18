#include <memory>

#include "Node.hpp"
#include "Leaf.hpp"

namespace bt {

Leaf::Leaf(std::string name, Blackboard::Ptr blackboard) {
    setProperties(blackboard);
    setName(name);
    robot = std::make_shared<roboteam_msgs::WorldRobot>();

}
void Leaf::setName(std::string newName) {
    name = std::move(newName);
}

std::shared_ptr<roboteam_msgs::WorldRobot> Leaf::getRobotFromProperties(bt::Blackboard::Ptr properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = (unsigned int) dealer::findRobotForRole(roleName);
        if (rtt::ai::World::getRobotForId(robotId, true)) {
            return rtt::ai::World::getRobotForId(robotId, true);
        }
        else {
            ROS_ERROR("%s Initialize -> robot %i does not exist in world", node_name().c_str(), robotId);
        }
    }
    else {
        ROS_ERROR("%s Initialize robot %i -> ROLE WAITING!!", node_name().c_str(), robotId);
    }
    return nullptr;
}

void Leaf::updateRobot() {
    if (rtt::ai::World::getRobotForId(robotId, true)) {
        robot = rtt::ai::World::getRobotForId(robotId, true);
    }
    else {
        ROS_ERROR("%s Update -> robot %i does not exist in world", node_name().c_str(), robotId);
    }
}
}
