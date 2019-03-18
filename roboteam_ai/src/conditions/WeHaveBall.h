//
// Created by robzelluf on 10/25/18.
//

#ifndef ROBOTEAM_AI_WEHAVEBALL_H
#define ROBOTEAM_AI_WEHAVEBALL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class WeHaveBall : public Condition {
    public:
        explicit WeHaveBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        std::string node_name() override { return "WeHaveBall"; }
};
} //ai
} //rtt



#endif //ROBOTEAM_AI_WEHAVEBALL_H
