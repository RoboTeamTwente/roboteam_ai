//
// Created by robzelluf on 1/21/19.
//

#ifndef ROBOTEAM_AI_ISBALLONOURSIDE_H
#define ROBOTEAM_AI_ISBALLONOURSIDE_H

#include "Condition.h"

namespace rtt {
namespace ai {

class IsBallOnOurSide : public Condition {
    FRIEND_TEST(IsBallOnOurSideTest, it_detects_ball_on_our_side);
private:
        bool inField = false;
    public:
        explicit IsBallOnOurSide(std::string name = "IsBallOnOurSide", bt::Blackboard::Ptr blackboard = nullptr);
        void onInitialize() override;
        Status onUpdate() override;
        std::string node_name() override;
};

} //ai
} //rtt


#endif //ROBOTEAM_AI_ISBALLONOURSIDE_H
