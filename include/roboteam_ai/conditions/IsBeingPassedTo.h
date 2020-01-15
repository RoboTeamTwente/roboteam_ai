#ifndef ROBOTEAM_AI_ISBEINGPASSEDTO_H
#define ROBOTEAM_AI_ISBEINGPASSEDTO_H

#include "Condition.h"

namespace rtt::ai {

    class IsBeingPassedTo : public Condition {
    public:
        explicit IsBeingPassedTo(std::string name = "IsBeingPassedTo", bt::Blackboard::Ptr blackboard = nullptr);

        Status onUpdate() override;
    };

} //rtt

#endif //ROBOTEAM_AI_ISBEINGPASSEDTO_H
