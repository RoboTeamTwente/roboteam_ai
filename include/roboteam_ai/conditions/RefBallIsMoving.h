//
// Created by mrlukasbos on 2-5-19.
//

#ifndef ROBOTEAM_AI_REFBALLISMOVING_H
#define ROBOTEAM_AI_REFBALLISMOVING_H

#include "Condition.h"

namespace rtt::ai {

class RefBallIsMoving : public Condition {
   public:
    explicit RefBallIsMoving(std::string name = "RefBallIsMoving", bt::Blackboard::Ptr blackboard = nullptr);

    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_REFBALLISMOVING_H
