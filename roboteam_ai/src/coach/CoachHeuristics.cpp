#include <utility>

//
// Created by robzelluf on 3/21/19.
//

#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

const double CoachHeuristics::MAX_DISTANCE_FROM_BALL = 6.0;
const double CoachHeuristics::CLOSE_TO_GOAL_WEIGHT = -0.1;
const double CoachHeuristics::SHOT_AT_GOAL_WEIGHT = -2.0;
const double CoachHeuristics::PASS_LINE_WEIGHT = -2.0;
const double CoachHeuristics::DISTANCE_TO_OPPONENTS_WEIGHT = -0.5;
const double CoachHeuristics::DISTANCE_FROM_CORNER_WEIGHT = -0.05;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(const Vector2 &position) {
    double distanceFromGoal = (world::field->get_their_goal_center() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT * distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free
double CoachHeuristics::calculateShotAtGoalScore(const Vector2& position, WorldData world) {
    double shortestDistance = control::ControlUtils::closestEnemyToLineDistance(position, world::field->get_their_goal_center(), std::move(world), false);

    return 1 - exp(SHOT_AT_GOAL_WEIGHT * shortestDistance);
}

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(const Vector2& position, const WorldData& world) {
    double shortestDistance = control::ControlUtils::closestEnemyToLineDistance(world.ball.pos, position, world, false);

    return 1 - exp(PASS_LINE_WEIGHT * shortestDistance);
}

/// Gives a higher score if the position is far away from enemy robots
double CoachHeuristics::calculateDistanceToOpponentsScore(const Vector2 &position, const WorldData& world) {
    Robot closestRobot = world::world->getRobotClosestToPoint(position, world::WhichRobots::THEIR_ROBOTS);
    if (closestRobot.id != -1) {
        double distance = (position - closestRobot.pos).length();
        return 1 - exp(DISTANCE_TO_OPPONENTS_WEIGHT * distance);
    } else {
        return 1;
    }
}

/// Gives a lower score if the position is too close to the corner of the field
double CoachHeuristics::calculateDistanceFromCornerScore(const Vector2& position, const roboteam_msgs::GeometryFieldSize& field) {
    Vector2 corner;
    corner.x = field.field_length / 2;
    if (position.y > 0) {
        corner.y = field.field_width / 2;
    } else {
        corner.y = -field.field_width / 2;
    }
    double distanceFromCorner = (position - corner).length();
    return 1 - exp(DISTANCE_FROM_CORNER_WEIGHT * distanceFromCorner);
}

/// Gives a higher score if the ball is not too far or too close to the ball (parabolic using maxDistanceFromBall)
double CoachHeuristics::calculateDistanceFromBallScore(const Vector2& position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball) {
    double distanceFromBall = (position - ball.pos).length();
    return -pow(distanceFromBall / (0.5 * MAX_DISTANCE_FROM_BALL), 2) + 2 * (distanceFromBall / (0.5 * MAX_DISTANCE_FROM_BALL));
}

/// Calculates a total score based on all the sub-scores
double CoachHeuristics::calculatePositionScore(const Vector2& position) {
    WorldData world = world::world->getWorld();
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();

    double closeToGoalScore = calculateCloseToGoalScore(position);
    double shotAtGoalScore = calculateShotAtGoalScore(position, world);
    double passLineScore = calculatePassLineScore(position, world);

    double score = shotAtGoalScore + passLineScore + closeToGoalScore;
    return score;
}

double CoachHeuristics::calculatePassScore(const Vector2 &position) {
    WorldData world = world::world->getWorld();
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    double closeToGoalScore = calculateCloseToGoalScore(position);
    double shotAtGoalScore = calculateShotAtGoalScore(position, world);
    double passLineScore = calculatePassLineScore(position, world);

    double score = closeToGoalScore + 3 * shotAtGoalScore + 2 * passLineScore;
    return score;
}

}
}
}
