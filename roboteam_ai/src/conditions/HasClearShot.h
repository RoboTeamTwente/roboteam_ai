#ifndef ROBOTEAM_AI_SHOTATGOAL_H
#define ROBOTEAM_AI_SHOTATGOAL_H

#include "Condition.h"
#include "../coach/GeneralPositionCoach.h"

namespace rtt {
namespace ai {

class HasClearShot : public Condition {
private:
    const double MAX_SHOOTING_DISTANCE = 3.0;
public:
    explicit HasClearShot(std::string name = "HasClearShot", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_SHOTATGOAL_H
