//
// Created by robzelluf on 3/22/19.
//

#include "Condition.h"
#include "roboteam_ai/src/coach/BallplacementCoach.h"

#ifndef ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H
#define ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H

namespace rtt {
namespace ai {

class TwoRobotBallPlacement : public Condition {
private:
    const double MAX_ONE_ROBOT_BALLPLACEMENT_DIST_TO_TARGET = 2.0;
public:
    explicit TwoRobotBallPlacement(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
    std::string node_name() override { return "TwoRobotBallPlacement"; }
};

}
}


#endif //ROBOTEAM_AI_TWOROBOTBALLPLACEMENT_H
