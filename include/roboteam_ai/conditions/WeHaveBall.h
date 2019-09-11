#ifndef ROBOTEAM_AI_WEHAVEBALL_H
#define ROBOTEAM_AI_WEHAVEBALL_H

#include "Condition.h"

namespace rtt {
namespace ai {

class WeHaveBall : public Condition {
    public:
        explicit WeHaveBall(std::string name = "WeHaveBall", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
};

} //ai
} //rtt



#endif //ROBOTEAM_AI_WEHAVEBALL_H
