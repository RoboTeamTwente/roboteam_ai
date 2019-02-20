//
// Created by rolf on 20-2-19.
//

#ifndef ROBOTEAM_AI_BALLNEAROURGOALLINEANDSTILL_H
#define ROBOTEAM_AI_BALLNEAROURGOALLINEANDSTILL_H
#include "Condition.h"
#include <roboteam_ai/src/utilities/Constants.h>
#include "../utilities/Field.h"

namespace rtt {
namespace ai {
class BallNearOurGoalLineAndStill : public Condition {
    private:
        double margin = Constants::ROBOT_RADIUS()*1.2;
    public:
        explicit BallNearOurGoalLineAndStill(std::string
        name = "BallNearOurGoalLineAndStill", bt::Blackboard::Ptr
        blackboard = nullptr
        );
        void initialize() override;
        Status update() override;
        std::string node_name() override;
};
};
}


#endif //ROBOTEAM_AI_BALLNEAROURGOALLINEANDSTILL_H
