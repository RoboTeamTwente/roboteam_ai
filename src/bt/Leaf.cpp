#include <memory>
#include <roboteam_ai/src/world/World.h>

#include "include/roboteam_ai/bt/Leaf.hpp"
#include "include/roboteam_ai/utilities/RobotDealer.h"
#include "ros/ros.h"
#include "include/roboteam_ai/world/WorldData.h"
#include "include/roboteam_ai/world/Robot.h"
#include "include/roboteam_ai/world/Ball.h"

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
    if (rtt::ai::world::world->getRobotForId(robotId, true)) {
        robot = rtt::ai::world::world->getRobotForId(robotId, true);
    }
    else {
        ROS_ERROR("%s Update -> robot %i does not exist in world", node_name().c_str(), robotId);
        robot = nullptr;
    }
}

void Leaf::terminate(Node::Status status) {
    robotId = - 1;
}

}
