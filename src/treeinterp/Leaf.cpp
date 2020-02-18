#include "include/roboteam_ai/treeinterp/Leaf.h"

#include <memory>

#include "utilities/RobotDealer.h"
#include <world_new/World.hpp>

namespace bt {

Leaf::Leaf(std::string name, Blackboard::Ptr blackboard) : name(std::move(name)) {
    setProperties(blackboard);
}

std::optional<rtt::world_new::view::RobotView> Leaf::getRobotFromProperties(const bt::Blackboard::Ptr& properties) {
    // If the robot does not have an assigned role, don't return a robot
    if (!properties->hasString("ROLE")) {
        std::cerr << "[Leaf::getRobotFromProperties]" << node_name().c_str()
                  << "Initialize -> robot " << robotId << " -> ROLE WAITING!!" << std::endl;
        return std::nullopt;
    }

    // Get the robot based on its role
    std::string roleName = properties->getString("ROLE");
    robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
    std::optional<rtt::world_new::view::RobotView> bot = world->getRobotForId(robotId, true);

    // If no robot with the role has been found in the world, something is going wrong
    if(!bot.has_value())
        std::cerr << "[Leaf::getRobotFromProperties]" << node_name().c_str()
                  << " Initialize -> robot " << robotId << " does not exist in world" << std::endl;
    return bot;
}

void Leaf::updateRobot() {
    robot = world->getRobotForId(robotId, true);
    if(!robot.has_value())
        std::cerr << "[Leaf::updateRobot]" << node_name().c_str()
                  << "Update -> robot " << robotId << " does not exist in world" << std::endl;
}

void Leaf::terminate(Node::Status status) { robotId = -1; }

}  // namespace bt
