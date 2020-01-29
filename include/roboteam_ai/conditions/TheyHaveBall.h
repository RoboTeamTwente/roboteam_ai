#ifndef ROBOTEAM_AI_THEYHAVEBALL_H
#define ROBOTEAM_AI_THEYHAVEBALL_H

#include "Condition.h"

namespace rtt::ai {

class TheyHaveBall : public Condition {
 public:
  explicit TheyHaveBall(std::string name = "TheyHaveBall", bt::Blackboard::Ptr blackboard = nullptr);
  Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_THEYHAVEBALL_H
