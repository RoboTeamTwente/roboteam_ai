//
// Created by robzelluf on 10/18/18.
//

#ifndef ROBOTEAM_AI_ISROBOTCLOSESTTOBALL_H
#define ROBOTEAM_AI_ISROBOTCLOSESTTOBALL_H

#include "Condition.h"
#include "roboteam_msgs/World.h"
#include  <boost/optional.hpp>
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class IsRobotClosestToBall : public rtt::ai::Condition {
public:
    IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
    Status Update();
};

}
}

#endif //ROBOTEAM_AI_ISROBOTCLOSESTTOBALL_H
