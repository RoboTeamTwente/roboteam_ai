//
// Created by robzelluf on 4/17/19.
//

#include "PassScore.h"

namespace rtt {
namespace ai {
namespace coach {

double PassScore::calculatePassScore(const Vector2 &position) {
    WorldData world = world::world->getWorld();
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    double closeToGoalScore = CoachHeuristics::calculateCloseToGoalScore(position);
    double shotAtGoalScore = CoachHeuristics::calculateShotAtGoalScore(position, world);
    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double behindBallScore = CoachHeuristics::calculateBehindBallScore(position, world);
    double distanceToOpponentScore = CoachHeuristics::calculateDistanceToOpponentsScore(position, world);

    double score =  CLOSE_TO_GOAL_WEIGHT * closeToGoalScore + 
                    SHOT_AT_GOAL_WEIGHT * shotAtGoalScore + 
                    PASS_LINE_WEIGHT * passLineScore + 
                    BEHIND_BALL_WEIGHT * behindBallScore + 
                    DISTANCE_TO_OPPONENT_WEIGHT * distanceToOpponentScore;
    
    return score;
}

}
}
}
