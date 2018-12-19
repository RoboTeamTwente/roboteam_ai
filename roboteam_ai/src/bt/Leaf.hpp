#pragma once

#include "Node.hpp"

namespace bt {

class Leaf : public Node {
    public:
        Leaf() = default;
        Leaf(std::string name, Blackboard::Ptr blackboard);
        void setName(std::string);
        std::string name;

    protected:
        std::shared_ptr<roboteam_msgs::WorldRobot> getRobotFromProperties(bt::Blackboard::Ptr properties);
        void updateRobot();
        std::shared_ptr<roboteam_msgs::WorldRobot> robot;
        int robotId = - 1;
};

}
