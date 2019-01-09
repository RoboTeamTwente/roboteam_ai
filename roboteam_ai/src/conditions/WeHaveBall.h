//
// Created by robzelluf on 10/25/18.
//

#ifndef ROBOTEAM_AI_WEHAVEBALL_H
#define ROBOTEAM_AI_WEHAVEBALL_H

#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include "Condition.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class WeHaveBall : public Condition {
    public:
        WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        Status update() override;
        std::string node_name() override { return "WeHaveBall"; }

};
} //ai
} //rtt



#endif //ROBOTEAM_AI_WEHAVEBALL_H
