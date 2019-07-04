//
// Created by baris on 25-4-19.
//

#ifndef ROBOTEAM_AI_BALLISCLOSE_H
#define ROBOTEAM_AI_BALLISCLOSE_H

#include "Condition.h"
namespace rtt {
namespace ai {
class BallIsClose : public Condition {
    public:
        explicit BallIsClose(std::string name = "BallIsClose", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        double distance = 1.0;
};
}
}

#endif //ROBOTEAM_AI_BALLISCLOSE_H
