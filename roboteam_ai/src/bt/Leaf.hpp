#pragma once

#include "Node.hpp"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"

namespace rtt {
namespace ai {
namespace world {
    class Robot;
    class Ball;
}
}
}

namespace bt {



class Leaf : public Node {
    public:
        Leaf() = default;
        Leaf(std::string name, Blackboard::Ptr blackboard);
        std::string name;

    protected:

        using RobotPtr = std::shared_ptr<rtt::ai::world::Robot>;
        using BallPtr = std::shared_ptr<rtt::ai::world::Ball>;

        std::shared_ptr<rtt::ai::world::Robot> getRobotFromProperties(bt::Blackboard::Ptr properties);
        void terminate(Status status) override;
        void updateRobot();
        std::shared_ptr<rtt::ai::world::Robot> robot;
        std::shared_ptr<rtt::ai::world::Ball> ball;
        int robotId = - 1;
};

}
