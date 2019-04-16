#ifndef ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
#define ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H

#include "../conditions/Condition.h"

namespace rtt {
namespace ai {

class IsRobotClosestToBall : public Condition {
    public:
        IsRobotClosestToBall(std::string name = "IsRobotClosestToBall", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_ISROBOTCLOSETSTOBALL_H
