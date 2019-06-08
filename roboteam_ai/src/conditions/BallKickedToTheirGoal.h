#ifndef ROBOTEAM_AI_BALLKICKEDTOTHEIRGOAL_H
#define ROBOTEAM_AI_BALLKICKEDTOTHEIRGOAL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class BallKickedToTheirGoal : public Condition {
private:
    const double BALL_TO_GOAL_MARGIN = 0.12; // m
    const double BALL_TO_GOAL_TIME = 2.5; // s
public:
    explicit BallKickedToTheirGoal(std::string name = "BallKickedToTheirGoal", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_BALLKICKEDTOTHEIRGOAL_H
