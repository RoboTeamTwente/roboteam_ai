//
// Created by robzelluf on 4/17/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVESCORE_H
#define ROBOTEAM_AI_OFFENSIVESCORE_H

#include "CoachHeuristics.h"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include <include/roboteam_ai/world/FieldMessage.h>

namespace rtt::ai::coach {


    class OffensiveScore {
    private:
        constexpr static double CLOSE_TO_GOAL_WEIGHT = 1.0;
        constexpr static double SHOT_AT_GOAL_WEIGHT = 1.0;
        constexpr static double PASS_LINE_WEIGHT = 2.0;
        constexpr static double DISTANCE_TO_OPPONENT_WEIGHT = 1.0;
        constexpr static double DISTANCE_TO_BALL_WEIGHT = 1.0;
        constexpr static double ANGLE_TO_GOAL_WEIGHT = 3.0;

        constexpr static double ZONE_RADIUS = 1.06;

        bool positionIsValid(const Vector2 &defaultZoneLocation, const Vector2 &positionToCheck);

    public:
        using WorldData = world::WorldData;

        double calculateOffensivePositionScore(const Vector2 &zoneLocation, const Vector2 &position,
                                               const WorldData &world, const FieldMessage &field);
    };
} // rtt::ai::coach


#endif //ROBOTEAM_AI_OFFENSIVESCORE_H
