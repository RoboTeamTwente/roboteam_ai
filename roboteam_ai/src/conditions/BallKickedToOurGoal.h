//
// Created by rolf on 12/12/18.
//

#ifndef ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
#define ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class BallKickedToOurGoal : public Condition {
        const double BALL_TO_GOAL_MARGIN = 0.0215;
        const double BALL_TO_GOAL_TIME = 2.5;
    public:
        explicit BallKickedToOurGoal(std::string name = "BallKickedToOurGoal", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        std::string node_name() override { return "BallKickedToOurGoal"; };
};

}
}

#endif //ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
