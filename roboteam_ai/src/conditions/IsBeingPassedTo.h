//
// Created by robzelluf on 1/22/19.
//

#ifndef ROBOTEAM_AI_ISBEINGPASSEDTO_H
#define ROBOTEAM_AI_ISBEINGPASSEDTO_H

#include "Condition.h"
#include "../utilities/Coach.h"

namespace rtt {
namespace ai {

class IsBeingPassedTo : public Condition {
public:
    explicit IsBeingPassedTo(std::string name = "IsBeingPassedTo", bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    Status onUpdate() override;
    std::string node_name() override;
};

} //ai
} //rtt

#endif //ROBOTEAM_AI_ISBEINGPASSEDTO_H
