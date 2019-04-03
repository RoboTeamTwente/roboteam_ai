//
// Created by robzelluf on 3/21/19.
//

#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

const double CoachHeuristics::MAX_DISTANCE_FROM_BALL = 6.0;
const double CoachHeuristics::CLOSE_TO_GOAL_WEIGHT = -0.1;
const double CoachHeuristics::SHOT_AT_GOAL_WEIGHT = -1.0;
const double CoachHeuristics::PASS_LINE_WEIGHT = -1.0;
const double CoachHeuristics::DISTANCE_TO_OPPONENTS_WEIGHT = -0.5;
const double CoachHeuristics::DISTANCE_FROM_CORNER_WEIGHT = -0.05;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(const Vector2 &position) {
    double distanceFromGoal = (world::field->get_their_goal_center() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT * distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free
//TODO: Choose to not take the opponents keeper into account for this calculation
double CoachHeuristics::calculateShotAtGoalScore(const Vector2 &position, WorldData world) {
    double safeDistanceFactor = 3;
    while (safeDistanceFactor > 0) {
        if (control::ControlUtils::clearLine(position, world::field->get_their_goal_center(), world, safeDistanceFactor)) {
            break;
        }
        safeDistanceFactor -= 0.5;
    }

    return 1 - exp(SHOT_AT_GOAL_WEIGHT * safeDistanceFactor);
}

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(const Vector2 &position, WorldData world) {
    double safeDistanceFactor = 3;
    while (safeDistanceFactor > 0) {
        if (control::ControlUtils::clearLine(world.ball.pos, position, world, safeDistanceFactor)) {
            break;
        }
        safeDistanceFactor -= 0.5;
    }

    return 1 - exp(PASS_LINE_WEIGHT * safeDistanceFactor);
}

/// Gives a higher score if the position is far away from enemy robots
double CoachHeuristics::calculateDistanceToOpponentsScore(const Vector2 &position) {
    Robot closestRobot = world::world->getRobotClosestToPoint(position, world::WhichRobots::THEIR_ROBOTS);
    if (closestRobot.id != -1) {
        double distance = (position - closestRobot.pos).length();
        return 1 - exp(DISTANCE_TO_OPPONENTS_WEIGHT * distance);
    } else {
        return 1;
    }
}

/// Gives a lower score if the position is too close to the corner of the field
double CoachHeuristics::calculateDistanceFromCornerScore(const Vector2 &position, roboteam_msgs::GeometryFieldSize field) {
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
double CoachHeuristics::calculateDistanceFromBallScore(const Vector2 &position, roboteam_msgs::GeometryFieldSize &field, BallPtr &ball) {
    double distanceFromBall = (position - ball->pos).length();
    return -pow(distanceFromBall / (0.5 * MAX_DISTANCE_FROM_BALL), 2) + 2 * (distanceFromBall / (0.5 * MAX_DISTANCE_FROM_BALL));
}

/// Calculates a total score based on all the sub-scores
double CoachHeuristics::calculatePositionScore(const Vector2 &position) {
    auto w = world::world->getWorld();
    auto ball = world::world->getBall();

    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    double closeToGoalScore = calculateCloseToGoalScore(position);
    double shotAtGoalScore = calculateShotAtGoalScore(position, w);
    double passLineScore = calculatePassLineScore(position, w);
    double closestOpponentScore = calculateDistanceToOpponentsScore(position);
    double distanceFromBallScore = calculateDistanceFromBallScore(position, field, ball);
    double behindBallScore = position.x < ball->pos.x ? 0.7 : 1.0;
    double distanceFromCornerScore = calculateDistanceFromCornerScore(position, field);

    double score = 2 * closeToGoalScore + 2 * shotAtGoalScore + passLineScore + closestOpponentScore
                   + distanceFromBallScore + behindBallScore + distanceFromCornerScore;

    return score;
}
    
}
}
}
