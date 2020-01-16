#ifndef ROBOTEAM_AI_ISBALLONOURSIDE_H
#define ROBOTEAM_AI_ISBALLONOURSIDE_H

#include "Condition.h"

namespace rtt::ai {

class IsBallOnOurSide : public Condition {
    FRIEND_TEST(IsBallOnOurSideTest, it_detects_ball_on_our_side);

   private:
    bool inField = false;

   public:
    explicit IsBallOnOurSide(std::string name = "IsBallOnOurSide", bt::Blackboard::Ptr blackboard = nullptr);

    void onInitialize() override;

    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ISBALLONOURSIDE_H
