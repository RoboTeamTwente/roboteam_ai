//
// Created by robzelluf on 4/17/19.
//

#include "coach/heuristics/PassScore.h"

#include "roboteam_proto/GeometryFieldSize.pb.h"
#include "world/Field.h"
#include "world/WorldData.h"

namespace rtt::ai::coach {

double PassScore::calculatePassScore(const Vector2 &position) {
    WorldData world = world::world->getWorld();
    FieldMessage field = world::field->get_field();
    double closeToGoalScore = CoachHeuristics::calculateCloseToGoalScore(position);
    double shotAtGoalScore = CoachHeuristics::calculateShotAtGoalScore(position, world);
    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double behindBallScore = CoachHeuristics::calculateBehindBallScore(position, world);
    double distanceToOpponentScore = CoachHeuristics::calculateDistanceToOpponentsScore(position);
    double distanceToBallScore = CoachHeuristics::calculatePassDistanceToBallScore(position, world);

    double score = CLOSE_TO_GOAL_WEIGHT * closeToGoalScore + SHOT_AT_GOAL_WEIGHT * shotAtGoalScore + PASS_LINE_WEIGHT * passLineScore + BEHIND_BALL_WEIGHT * behindBallScore +
                   DISTANCE_TO_OPPONENT_WEIGHT * distanceToOpponentScore + DISTANCE_FROM_BALL_WEIGHT * distanceToBallScore;

    return score;
}

}  // namespace rtt::ai::coach
