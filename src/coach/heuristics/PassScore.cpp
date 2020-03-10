//
// Created by robzelluf on 4/17/19.
//

#include <include/roboteam_ai/world_new/World.hpp>
#include <roboteam_utils/Print.h>
#include "coach/heuristics/PassScore.h"
#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "world/FieldComputations.h"

namespace rtt::ai::coach {

double PassScore::calculatePassScore(const Field &field, const Vector2 &position) {
    auto worldOpt = world_new::World::instance()->getWorld();
    if (!worldOpt.has_value()) {
        RTT_WARNING("No world available!")
        return 0;
    }
    auto world = worldOpt.value();

    double closeToGoalScore = CoachHeuristics::calculateCloseToGoalScore(field, position);
    double shotAtGoalScore = CoachHeuristics::calculateShotAtGoalScore(field, position, world);
    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double behindBallScore = CoachHeuristics::calculateBehindBallScore(position, world);
    double distanceToOpponentScore = CoachHeuristics::calculateDistanceToOpponentsScore(position);
    double distanceToBallScore = CoachHeuristics::calculatePassDistanceToBallScore(field, position, world);

    double score = CLOSE_TO_GOAL_WEIGHT * closeToGoalScore + SHOT_AT_GOAL_WEIGHT * shotAtGoalScore + PASS_LINE_WEIGHT * passLineScore + BEHIND_BALL_WEIGHT * behindBallScore +
                   DISTANCE_TO_OPPONENT_WEIGHT * distanceToOpponentScore + DISTANCE_FROM_BALL_WEIGHT * distanceToBallScore;

    return score;
}

}  // namespace rtt::ai::coach
