//
// Created by robzelluf on 4/17/19.
//

#include "OffensiveScore.h"

namespace rtt {
namespace ai {
namespace coach {

OffensiveScore g_offensiveScore;

/// Calculates a total score based on all the sub-scores
double OffensiveScore::calculateOffensivePositionScore(const Vector2 &position) {
    WorldData world = world::world->getWorld();
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();

    double closeToGoalScore = CoachHeuristics::calculateCloseToGoalScore(position);
    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double shotAtGoalScore = CoachHeuristics::calculateShotAtGoalScore(position, world);
    double distanceToOpponentScore = CoachHeuristics::calculateDistanceToOpponentsScore(position, world);

    double score =  SHOT_AT_GOAL_WEIGHT * shotAtGoalScore +
                    PASS_LINE_WEIGHT * passLineScore +
                    CLOSE_TO_GOAL_WEIGHT * closeToGoalScore +
                    DISTANCE_TO_OPPONENT_WEIGHT * distanceToOpponentScore;
    return score;
}

}
}
}
