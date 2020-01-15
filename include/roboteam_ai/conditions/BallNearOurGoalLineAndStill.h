#ifndef ROBOTEAM_AI_BALLNEAROURGOALLINEANDSTILL_H
#define ROBOTEAM_AI_BALLNEAROURGOALLINEANDSTILL_H

#include "Condition.h"
#include "utilities/Constants.h"
#include "world/Field.h"

namespace rtt::ai {

    class BallNearOurGoalLineAndStill : public Condition {
    private:
        double margin = Constants::CLOSE_TO_BORDER_DISTANCE();
    public:
        explicit BallNearOurGoalLineAndStill(std::string name = "BallNearOurGoalLineAndStill",
                                             bt::Blackboard::Ptr blackboard = nullptr);

        void onInitialize() override;

        Status onUpdate() override;
    };

} // rtt

#endif //ROBOTEAM_AI_BALLNEAROURGOALLINEANDSTILL_H
