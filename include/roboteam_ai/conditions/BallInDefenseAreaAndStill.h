#ifndef ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
#define ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H

#include "Condition.h"

namespace rtt::ai {

    class BallInDefenseAreaAndStill : public Condition {
        FRIEND_TEST(DetectsDefenseArea, BallInDefenseAreaAndStill);

        private:
        bool theirDefenceArea;
        bool outsideField = false;

        public:
        explicit BallInDefenseAreaAndStill(std::string name = "BallInDefenseAreaAndStill", bt::Blackboard::Ptr blackboard = nullptr);

        void onInitialize() override;

        Status onUpdate() override;
    };

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLINDEFENSEAREAANDSTILL_H
