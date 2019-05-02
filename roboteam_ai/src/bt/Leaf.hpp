#pragma once

#include "Node.hpp"
#include "roboteam_ai/src/world/WorldData.h"
#include "roboteam_ai/src/world/World.h"
#include "roboteam_ai/src/world/Field.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"

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

        std::shared_ptr<rtt::ai::world::Robot> getRobotFromProperties(bt::Blackboard::Ptr properties);
        void terminate(Status status) override;
        void updateRobot();
        RobotPtr robot;
        BallPtr ball;
        int robotId = - 1;
};

}
