//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
#define ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H

#include "Condition.h"

namespace rtt {
namespace ai {
class BallKickedToOurGoal : public Condition {
    public:
        explicit BallKickedToOurGoal(std::string name = "BallKickedToOurGoal",
                bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
        std::string node_name() override;
};

}
}

#endif //ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
