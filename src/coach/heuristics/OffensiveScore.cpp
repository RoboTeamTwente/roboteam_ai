//
// Created by robzelluf on 4/17/19.
//

#include "coach/heuristics/OffensiveScore.h"
#include "coach/OffensiveCoach.h"
#include "world/FieldComputations.h"

namespace rtt {
namespace ai {
namespace coach {

OffensiveScore g_offensiveScore;

/// Calculates a total score based on all the sub-scores
double OffensiveScore::calculateOffensivePositionScore(const Vector2 &zoneLocation, const Vector2 &position,
        const WorldData &world, const FieldMessage &field) {

    if (!positionIsValid(zoneLocation, position)) return 0.0;
    double closeToGoalScore = CoachHeuristics::calculateCloseToGoalScore(position);
    double passLineScore = CoachHeuristics::calculatePassLineScore(position, world);
    double shotAtGoalScore = CoachHeuristics::calculateShotAtGoalScore(position, world);
    double distanceToOpponentScore = CoachHeuristics::calculateDistanceToOpponentsScore(position);
    double distanceToBallScore = CoachHeuristics::calculatePositionDistanceToBallScore(position, world);
    double angleToGoalScore = CoachHeuristics::calculateAngleToGoalScore(position);

    double score = SHOT_AT_GOAL_WEIGHT*shotAtGoalScore +
            PASS_LINE_WEIGHT*passLineScore +
            CLOSE_TO_GOAL_WEIGHT*closeToGoalScore +
            DISTANCE_TO_OPPONENT_WEIGHT*distanceToOpponentScore +
            DISTANCE_TO_BALL_WEIGHT*distanceToBallScore +
            ANGLE_TO_GOAL_WEIGHT*angleToGoalScore;
    return score;
}

/// Check if a position being checked is not outside field, within the correct zone, etc
bool OffensiveScore::positionIsValid(const Vector2 &defaultZoneLocation, const Vector2 &positionToCheck) {
    // check if the offender is not blocking the goal
    FieldMessage field = FieldMessage::get_field();
    std::vector<Vector2> vertices;
    auto goalSides = FieldComputations::getGoalSides(field, false);
    vertices.push_back(goalSides.start);
    vertices.push_back(goalSides.end);
    vertices.push_back(world::world->getBall()->getPos());
    Polygon goalBallTriangle(vertices);

    if(goalBallTriangle.contains(positionToCheck)) {
        return false;
    }

    // check if the point is in the field and out of the defense area
    if (! FieldComputations::pointIsInField(field, positionToCheck, Constants::ROBOT_RADIUS()*6) ||
        FieldComputations::pointIsInDefenceArea(field, positionToCheck, false, Constants::ROBOT_RADIUS()*2)) {
        return false;
    }

    // check if the point is out of the default zone location
    if ((positionToCheck - defaultZoneLocation).length2() > ZONE_RADIUS * ZONE_RADIUS) return false;

    // check if the point is closer to another zone
    for (auto &otherDefaultPosition : g_offensiveCoach.getZoneLocations()) {
        if (otherDefaultPosition != defaultZoneLocation &&
                (otherDefaultPosition - positionToCheck).length2()
                        < (defaultZoneLocation - positionToCheck).length2()) {
            return false;
        }
    }

}

}
}
}
