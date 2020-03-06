#include "include/roboteam_ai/treeinterp/Leaf.h"
#include <memory>
#include <roboteam_utils/Print.h>
#include "utilities/RobotDealer.h"
#include <world_new/World.hpp>

namespace bt {

Leaf::Leaf(std::string name, Blackboard::Ptr blackboard) : name(std::move(name)) {
    setProperties(blackboard);
}

std::optional<rtt::world_new::view::RobotView> Leaf::getRobotFromProperties(const bt::Blackboard::Ptr& properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);

        if (world->getRobotForId(robotId, true)) {
            if (robotId == -1) {
                RTT_WARNING("getting robot for id with id = -1!!!")
            }
            return rtt::world_new::World::instance()->getWorld()->getRobotForId(robotId, true);
        } else {
            RTT_WARNING(node_name(), " Initialize -> robot ", robotId, " does not exist in world")
        }
    } else {
        RTT_WARNING(node_name(), " Initialize -> robot ", robotId, " role status: waiting!")
    }
    return std::optional<rtt::world_new::view::RobotView>(nullptr);
}

void Leaf::updateRobot() {
    if (rtt::world_new::World::instance()->getWorld()->getRobotForId(robotId, true)) {
        robot = rtt::world_new::World::instance()->getWorld()->getRobotForId(robotId, true);
    } else {
        RTT_WARNING(node_name(), " Update -> robot ", robotId, " does not exist in world!")
        robot = std::optional<rtt::world_new::view::RobotView>(nullptr);
    }
}

void Leaf::terminate(Node::Status status) { robotId = -1; }

}  // namespace bt
