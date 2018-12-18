//
// Created by mrlukasbos on 11-12-18.
//

#ifndef ROBOTEAM_AI_CANSEEGOAL_H
#define ROBOTEAM_AI_CANSEEGOAL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class CanSeeGoal : public Condition {
    public:
        explicit CanSeeGoal(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        using Status = bt::Node::Status;
        Status update() override;
        std::string node_name() override { return "HasBall"; }
};

} // ai
} // rtt
#endif //ROBOTEAM_AI_CANSEEGOAL_H
