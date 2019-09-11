#ifndef ROBOTEAM_AI_ISBEINGPASSEDTO_H
#define ROBOTEAM_AI_ISBEINGPASSEDTO_H

#include "Condition.h"

namespace rtt {
namespace ai {

class IsBeingPassedTo : public Condition {
public:
    explicit IsBeingPassedTo(std::string name = "IsBeingPassedTo", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_ISBEINGPASSEDTO_H
