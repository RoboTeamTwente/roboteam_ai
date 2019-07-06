//
// Created by roboteam on 6/07/19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTLINEISCLOSE_H
#define ROBOTEAM_AI_BALLPLACEMENTLINEISCLOSE_H

#include "Condition.h"
namespace rtt {
namespace ai {
class BallPlacementLineIsClose : public Condition {
    public:
        explicit BallPlacementLineIsClose(std::string name = "BallIsClose", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        double distance = 1.0;
};
}
}

#endif //ROBOTEAM_AI_BALLPLACEMENTLINEISCLOSE_H
