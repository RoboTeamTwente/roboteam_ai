#ifndef ROBOTEAM_AI_IHAVEBALL_HPP
#define ROBOTEAM_AI_IHAVEBALL_HPP

#include "Condition.h"

namespace rtt::ai {

class HasBall : public Condition {
   public:
    explicit HasBall(std::string name = "HasBall", bt::Blackboard::Ptr blackboard = nullptr);

    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_IHAVEBALL_HPP
