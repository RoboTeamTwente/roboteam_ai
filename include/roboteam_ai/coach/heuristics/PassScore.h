//
// Created by robzelluf on 4/17/19.
//

#ifndef ROBOTEAM_AI_PASSSCORE_H
#define ROBOTEAM_AI_PASSSCORE_H

#include "CoachHeuristics.h"

namespace rtt::ai::coach {

class PassScore {
   private:
    const double CLOSE_TO_GOAL_WEIGHT = 2.0;
    const double SHOT_AT_GOAL_WEIGHT = 1.0;
    const double PASS_LINE_WEIGHT = 6.0;
    const double BEHIND_BALL_WEIGHT = 2.0;
    const double DISTANCE_TO_OPPONENT_WEIGHT = 1.0;
    const double DISTANCE_FROM_BALL_WEIGHT = 2.0;

   public:
    using WorldData = world::WorldData;
    double calculatePassScore(const Field &field, const Vector2 &position);
};

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_PASSSCORE_H
