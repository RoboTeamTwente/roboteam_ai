//
// Created by robzelluf on 10/18/18.
//

#ifndef ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
#define ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H

#include "../conditions/Condition.h"

namespace rtt {
namespace ai {

class IsRobotClosestToBall : public Condition {
    public:
        IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        std::string node_name() override { return "IsRobotClosestToBall";}
};

}
}

#endif //ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
