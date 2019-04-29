//
// Created by baris on 25-4-19.
//

#ifndef ROBOTEAM_AI_BALLNOTTOOCLOSE_H
#define ROBOTEAM_AI_BALLNOTTOOCLOSE_H

#include "Condition.h"
namespace rtt {
namespace ai {
class BallNotTooClose : public Condition {
    public:
        explicit BallNotTooClose(std::string name = "BallNotTooClose", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        double distance = 0.5;
};
}
}

#endif //ROBOTEAM_AI_BALLNOTTOOCLOSE_H
