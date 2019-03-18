//
// Created by robzelluf on 10/24/18.
//

#ifndef ROBOTEAM_AI_THEYHAVEBALL_H
#define ROBOTEAM_AI_THEYHAVEBALL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class TheyHaveBall : public Condition {
    public:
        explicit TheyHaveBall(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        std::string node_name() override { return "TheyHaveBall"; }
};
}
}

#endif //ROBOTEAM_AI_THEYHAVEBALL_H
