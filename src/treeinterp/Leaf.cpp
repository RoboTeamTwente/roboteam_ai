#include "include/roboteam_ai/treeinterp/Leaf.h"

#include <memory>

#include "utilities/RobotDealer.h"
#include <world_new/World.hpp>

namespace bt {

Leaf::Leaf(std::string name, Blackboard::Ptr blackboard) : name(std::move(name)) {
    setProperties(blackboard);
}

std::optional<rtt::world_new::view::RobotView> Leaf::getRobotFromProperties(bt::Blackboard::Ptr properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
        std::optional<rtt::world_new::view::RobotView> robot = world->getRobotForId(robotId, true);

        if(robotId == -1)
            std::cout << "[Leaf::getRobotFromProperties] Warning! Getting robot for id = -1 !" << std::endl;
        if(!robot.has_value())
            std::cerr << "[Leaf::getRobotFromProperties]" << node_name().c_str()
                      << " Initialize -> robot " << robotId << " does not exist in world" << std::endl;
        return robot;

    } else {
        std::cerr << "[Leaf::getRobotFromProperties]" << node_name().c_str()
                  << "Initialize -> robot " << robotId << " -> ROLE WAITING!!" << std::endl;
    }
    return std::nullopt;
}

void Leaf::updateRobot() {
    robot = world->getRobotForId(robotId, true);
    if(!robot.has_value())
        std::cerr << "[Leaf::updateRobot]" << node_name().c_str()
                  << "Update -> robot " << robotId << " does not exist in world" << std::endl;
}

void Leaf::terminate(Node::Status status) { robotId = -1; }

}  // namespace bt
