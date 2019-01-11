//
// Created by rolf on 22-10-18.
//

#ifndef ROBOTEAM_AI_CANREACHPOINT_HPP
#define ROBOTEAM_AI_CANREACHPOINT_HPP

#include "Condition.h"

namespace rtt {
namespace ai {

class CanReachPoint : public Condition {
    public:
        CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard);
        double estimateTimeToPoint(Vector2 currentPos, Vector2 currentVel, Vector2 targetPos);
        Status update() override;
        std::string node_name() override { return "CanReachPoint"; }
};
}
}

#endif //ROBOTEAM_AI_CANREACHPOINT_HPP
