//
// Created by baris on 22/10/18.
//

#ifndef ROBOTEAM_AI_ISINZONE_H
#define ROBOTEAM_AI_ISINZONE_H

#include "Condition.h"
#include "../utilities/World.h"

namespace rtt {
namespace ai {

class IsInZone : public Condition {
    public:
        explicit IsInZone(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
};

} // ai
} //rtt

#endif //ROBOTEAM_AI_ISINZONE_H
