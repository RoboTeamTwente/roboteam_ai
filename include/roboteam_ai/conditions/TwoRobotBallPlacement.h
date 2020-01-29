#include "Condition.h"
#include "coach/BallplacementCoach.h"

#ifndef ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H
#define ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H

namespace rtt::ai {

class TwoRobotBallPlacement : public Condition {
 public:
  explicit TwoRobotBallPlacement(std::string name = "TwoRobotBallPlacement", bt::Blackboard::Ptr blackboard = nullptr);
  Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H
