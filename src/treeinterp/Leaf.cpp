#include "include/roboteam_ai/treeinterp/Leaf.h"

#include <memory>

#include <world/World.h>
#include <roboteam_utils/Print.h>
#include "utilities/RobotDealer.h"
#include "world/Ball.h"
#include "world/Robot.h"
#include "world/WorldData.h"

namespace bt {

Leaf::Leaf(std::string name, Blackboard::Ptr blackboard) : name(std::move(name)) {
    setProperties(blackboard);
    robot = std::make_shared<rtt::ai::world::Robot>(rtt::ai::world::Robot());
    ball = std::make_shared<rtt::ai::world::Ball>(rtt::ai::world::Ball());
}

std::shared_ptr<rtt::ai::world::Robot> Leaf::getRobotFromProperties(const bt::Blackboard::Ptr& properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
        if (rtt::ai::world::world->getRobotForId(robotId, true)) {
            if (robotId == -1) {
                rtt_warning("getting robot for id with id = -1!!!")
            }
            return rtt::ai::world::world->getRobotForId(robotId, true);
        } else {
            rtt_warning(node_name() + " Initialize -> robot " + std::to_string(robotId) + " does not exist in world")
        }
    } else {
        rtt_warning(node_name() + " Initialize -> robot " + std::to_string(robotId) + " role status: waiting!")
    }
    return nullptr;
}

void Leaf::updateRobot() {
    if (rtt::ai::world::world->getRobotForId(robotId, true)) {
        robot = rtt::ai::world::world->getRobotForId(robotId, true);
    } else {
        rtt_warning(node_name() + " Update -> robot " + std::to_string(robotId) + " does not exist in world!")
        robot = nullptr;
    }
}

void Leaf::terminate(Node::Status status) { robotId = -1; }

}  // namespace bt
