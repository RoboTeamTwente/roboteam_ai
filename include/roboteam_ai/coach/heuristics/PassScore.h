//
// Created by robzelluf on 4/17/19.
//

#ifndef ROBOTEAM_AI_PASSSCORE_H
#define ROBOTEAM_AI_PASSSCORE_H

#include "CoachHeuristics.h"

namespace rtt::ai::coach {
    class PassScore {
    private:
        constexpr static double CLOSE_TO_GOAL_WEIGHT = 2.0;
        constexpr static double SHOT_AT_GOAL_WEIGHT = 1.0;
        constexpr static double PASS_LINE_WEIGHT = 6.0;
        constexpr static double BEHIND_BALL_WEIGHT = 2.0;
        constexpr static double DISTANCE_TO_OPPONENT_WEIGHT = 1.0;
        constexpr static double DISTANCE_FROM_BALL_WEIGHT = 2.0;

    public:
        using WorldData = world::WorldData;

        static double calculatePassScore(const Vector2 &position);
    };


}


#endif //ROBOTEAM_AI_PASSSCORE_H
