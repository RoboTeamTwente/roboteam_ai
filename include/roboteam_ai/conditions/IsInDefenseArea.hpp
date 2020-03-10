#ifndef ROBOTEAM_AI_ISINDEFENSEAREA_HPP
#define ROBOTEAM_AI_ISINDEFENSEAREA_HPP

#include "Condition.h"

namespace rtt::ai {

class IsInDefenseArea : public ai::Condition {
   private:
    using status = bt::Node::Status;
    bool ourDefenseArea;
    bool outsideField;
    Vector2 point;
    float margin;
    double secondsAhead;

   public:
    explicit IsInDefenseArea(std::string name = "IsInDefenseArea", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ISINDEFENSEAREA_HPP
