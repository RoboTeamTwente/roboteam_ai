#pragma once

#include "Node.hpp"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "roboteam_ai/src/world/WorldData.h"

namespace bt {

class Leaf : public Node {
    public:
        Leaf() = default;
        Leaf(std::string name, Blackboard::Ptr blackboard);
        std::string name;

    protected:
        using Robot = rtt::ai::world::Robot;
        using RobotPtr = std::shared_ptr<Robot>;
        using Ball = rtt::ai::world::Ball;
        using BallPtr = std::shared_ptr<Ball>;

        std::shared_ptr<roboteam_msgs::WorldRobot> getRobotFromProperties(bt::Blackboard::Ptr properties);
        void updateRobot();
        std::shared_ptr<roboteam_msgs::WorldRobot> robot;
        std::shared_ptr<roboteam_msgs::WorldBall> ball;
        int robotId = - 1;
};

}
