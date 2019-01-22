//
// Created by robzelluf on 1/21/19.
//

#ifndef ROBOTEAM_AI_ISBALLONOURSIDE_H
#define ROBOTEAM_AI_ISBALLONOURSIDE_H

#include "Condition.h"

namespace rtt {
namespace ai {

class IsBallOnOurSide : public Condition {
private:
        bool inField = true;
    public:
        explicit IsBallOnOurSide(std::string name = "IsBallOnOurSide", bt::Blackboard::Ptr blackboard = nullptr);
        void initialize() override;
        Status update() override;
        std::string node_name() override;
};

} //ai
} //rtt


#endif //ROBOTEAM_AI_ISBALLONOURSIDE_H
