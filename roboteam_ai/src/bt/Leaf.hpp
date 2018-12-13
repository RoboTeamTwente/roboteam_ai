#pragma once

#include "Node.hpp"

namespace bt {
//TODO: implement node_name() functionality? Can perhaps also be done elsewhere
class Leaf : public Node {
    public:
        Leaf() = default;
        virtual ~Leaf() = default;
        Leaf(std::string name, Blackboard::Ptr blackboard);
        virtual Status update() = 0;
        void setName(std::string);
        std::string name;

protected:
    std::shared_ptr<roboteam_msgs::WorldRobot> getRobotFromProperties(bt::Blackboard::Ptr properties);
    void updateRobot();
    std::shared_ptr<roboteam_msgs::WorldRobot> robot;
    int robotId = -1;
};

}
