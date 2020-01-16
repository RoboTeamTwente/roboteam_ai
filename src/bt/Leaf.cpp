#include <world/World.h>
#include <memory>

#include "bt/Leaf.hpp"
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

std::shared_ptr<rtt::ai::world::Robot> Leaf::getRobotFromProperties(bt::Blackboard::Ptr properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
        if (rtt::ai::world::world->getRobotForId(robotId, true)) {
            if (robotId == -1) std::cout << "getting robot for id with id = -1!!!" << std::endl;
            return rtt::ai::world::world->getRobotForId(robotId, true);
        } else {
            std::cerr << node_name().c_str() << " Initialize -> robot " << robotId << " does not exist in world" << std::endl;
        }
    } else {
        std::cerr << node_name().c_str() << "Initialize -> robot " << robotId << " -> ROLE WAITING!!" << std::endl;
    }
    return nullptr;
}

void Leaf::updateRobot() {
    if (rtt::ai::world::world->getRobotForId(robotId, true)) {
        robot = rtt::ai::world::world->getRobotForId(robotId, true);
    } else {
        std::cerr << node_name().c_str() << "Update -> robot " << robotId << " does not exist in world" << std::endl;
        robot = nullptr;
    }
}

void Leaf::terminate(Node::Status status) { robotId = -1; }

}  // namespace bt
