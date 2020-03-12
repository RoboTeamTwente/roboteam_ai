//
// Created by robzelluf on 4/17/19.
//

#ifndef ROBOTEAM_AI_OFFENSIVESCORE_H
#define ROBOTEAM_AI_OFFENSIVESCORE_H

#include "CoachHeuristics.h"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "world/Field.h"

namespace rtt::ai::coach {

class OffensiveScore {
   private:
    const double CLOSE_TO_GOAL_WEIGHT = 1.0;
    const double SHOT_AT_GOAL_WEIGHT = 1.0;
    const double PASS_LINE_WEIGHT = 2.0;
    const double DISTANCE_TO_OPPONENT_WEIGHT = 1.0;
    const double DISTANCE_TO_BALL_WEIGHT = 1.0;
    const double ANGLE_TO_GOAL_WEIGHT = 3.0;

    const double ZONE_RADIUS = 1.06;

    bool positionIsValid(const world::Field &field, const Vector2 &defaultZoneLocation, const Vector2 &positionToCheck);

   public:
    double calculateOffensivePositionScore(const Vector2 &zoneLocation, const Vector2 &position, world_new::view::WorldDataView world, const world::Field &field);
};

}  // namespace rtt::ai::coach

#endif  // ROBOTEAM_AI_OFFENSIVESCORE_H
