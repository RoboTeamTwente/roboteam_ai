#include <memory>
#include <roboteam_ai/src/world/World.h>

#include "Leaf.hpp"
#include "../utilities/RobotDealer.h"
#include "ros/ros.h"
#include "../world/WorldData.h"
#include "../world/Robot.h"
#include "../world/Ball.h"

namespace bt {

Leaf::Leaf(std::string name, Blackboard::Ptr blackboard)
        :name(std::move(name)) {
    setProperties(blackboard);
    robot = std::make_shared<rtt::ai::world::Robot>(rtt::ai::world::Robot());
    ball = std::make_shared<rtt::ai::world::Ball>(rtt::ai::world::Ball());
}

std::shared_ptr<rtt::ai::world::Robot> Leaf::getRobotFromProperties(bt::Blackboard::Ptr properties) {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robotId = rtt::ai::robotDealer::RobotDealer::findRobotForRole(roleName);
        if (rtt::ai::world::world->getRobotForId(robotId, true)) {
            if (robotId == - 1) std::cout << "getting robot for id with id = -1!!!" << std::endl;
            return rtt::ai::world::world->getRobotForId(robotId, true);
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
    robot = rtt::ai::world::world->getRobotForId(robotId, true);
}

void Leaf::terminate(Node::Status status) {
    robotId = - 1;
}

}
