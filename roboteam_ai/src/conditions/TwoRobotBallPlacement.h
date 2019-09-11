#include "Condition.h"
#include "roboteam_ai/src/coach/BallplacementCoach.h"

#ifndef ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H
#define ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H

namespace rtt {
namespace ai {

class TwoRobotBallPlacement : public Condition {
public:
    explicit TwoRobotBallPlacement(std::string name = "TwoRobotBallPlacement", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt


#endif //ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H
