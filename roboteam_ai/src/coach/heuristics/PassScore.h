//
// Created by robzelluf on 4/17/19.
//

#ifndef ROBOTEAM_AI_PASSSCORE_H
#define ROBOTEAM_AI_PASSSCORE_H

#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

class PassScore {
private:
    const double CLOSE_TO_GOAL_WEIGHT = 1.0;
    const double SHOT_AT_GOAL_WEIGHT = 1.0;
    const double PASS_LINE_WEIGHT = 2.0;
    const double BEHIND_BALL_WEIGHT = 1.0;
    const double DISTANCE_TO_OPPONENT_WEIGHT = 1.0;
    
public:
    using WorldData = world::WorldData;
    double calculatePassScore(const Vector2& position);
};

}
}
}


#endif //ROBOTEAM_AI_PASSSCORE_H
