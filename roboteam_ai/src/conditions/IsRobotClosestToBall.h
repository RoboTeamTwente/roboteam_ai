//
// Created by robzelluf on 10/18/18.
//

#ifndef ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
#define ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H

#include "../conditions/Condition.h"
#include "roboteam_msgs/World.h"
#include "roboteam_utils/LastWorld.h"

namespace rtt {
namespace ai {

class IsRobotClosestToBall : public Condition {
    public:
        IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        Status update() override;
        std::string node_name() override { return "IsRobotClosestToBall";}

    private:
        ros::NodeHandle n;
};

}
}

#endif //ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
