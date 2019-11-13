#ifndef ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
#define ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class BallKickedToOurGoal : public Condition {
private:
    constexpr static double BALL_TO_GOAL_MARGIN = 0.12; // m
    constexpr static double BALL_TO_GOAL_TIME = 2.5; // s
public:
    explicit BallKickedToOurGoal(std::string name = "BallKickedToOurGoal", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_BALLKICKEDTOOURGOAL_H
