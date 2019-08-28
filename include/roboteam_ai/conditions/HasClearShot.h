#ifndef ROBOTEAM_AI_SHOTATGOAL_H
#define ROBOTEAM_AI_SHOTATGOAL_H

#include "Condition.h"
#include "include/roboteam_ai/control/PositionUtils.h"

namespace rtt {
namespace ai {

class HasClearShot : public Condition {
private:
    const double FORCED_SHOOTING_DISTANCE = 2.5;
    const double MIN_VIEW_AT_GOAL = 0.1;
    const double MAX_SHOOTING_DISTANCE = 5.5;
public:
    explicit HasClearShot(std::string name = "HasClearShot", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_SHOTATGOAL_H
