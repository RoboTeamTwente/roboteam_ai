//
// Created by robzelluf on 4/17/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVESCORE_H
#define ROBOTEAM_AI_OFFENSIVESCORE_H

#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

class OffensiveScore {
private:
    const double CLOSE_TO_GOAL_WEIGHT = 1.0;
    const double SHOT_AT_GOAL_WEIGHT = 1.0;
    const double PASS_LINE_WEIGHT = 2.0;
    const double DISTANCE_TO_OPPONENT_WEIGHT = 1.0;
    const double DISTANCE_TO_BALL_WEIGHT = 1.0;

public:
    using WorldData = world::WorldData;
    double calculateOffensivePositionScore(const Vector2 &position);
};

}
}
}


#endif //ROBOTEAM_AI_OFFENSIVESCORE_H
