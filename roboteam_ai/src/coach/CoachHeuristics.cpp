//
// Created by robzelluf on 3/21/19.
//

#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

double MAX_DISTANCE_FROM_BALL = 6.0;
double CLOSE_TO_GOAL_WEIGHT = -0.1;
double SHOT_AT_GOAL_WEIGHT = -1.0;
double PASS_LINE_WEIGHT = -1.0;
double DISTANCE_TO_OPPONENTS_WEIGHT = -0.5;
double DISTANCE_FROM_CORNER_WEIGHT = -0.05;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(Vector2 position) {
    double distanceFromGoal = (Field::get_their_goal_center() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT * distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free
//TODO: Choose to not take the opponents keeper into account for this calculation
double CoachHeuristics::calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world) {
    double safeDistanceFactor = 3;
    while (safeDistanceFactor > 0) {
        if (control::ControlUtils::clearLine(position, Field::get_their_goal_center(), world, safeDistanceFactor)) {
            break;
        }
        safeDistanceFactor -= 0.5;
    }

    return 1 - exp(SHOT_AT_GOAL_WEIGHT * safeDistanceFactor);
}

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(Vector2 position, roboteam_msgs::World world) {
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
double CoachHeuristics::calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world) {
    shared_ptr<roboteam_msgs::WorldRobot> closestRobot = World::getRobotClosestToPoint(world.them, position);
    if (closestRobot) {
        double distance = (position - closestRobot->pos).length();
        return 1 - exp(DISTANCE_TO_OPPONENTS_WEIGHT * distance);
    } else {
        return 1;
    }
}

/// Gives a lower score if the position is too close to the corner of the field
double CoachHeuristics::calculateDistanceFromCornerScore(Vector2 position, roboteam_msgs::GeometryFieldSize field) {
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
double CoachHeuristics::calculateDistanceFromBallScore(Vector2 position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball) {
    double distanceFromBall = (position - ball.pos).length();
    return -pow(distanceFromBall / (0.5 * MAX_DISTANCE_FROM_BALL), 2) + 2 * (distanceFromBall / (0.5 * MAX_DISTANCE_FROM_BALL));
}

/// Calculates a total score based on all the sub-scores
double CoachHeuristics::calculatePositionScore(Vector2 position) {
    roboteam_msgs::World world = World::get_world();
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double closeToGoalScore = calculateCloseToGoalScore(position);
    double shotAtGoalScore = calculateShotAtGoalScore(position, world);
    double passLineScore = calculatePassLineScore(position, world);
    double closestOpponentScore = calculateDistanceToOpponentsScore(position, world);
    double distanceFromBallScore = calculateDistanceFromBallScore(position, field, world.ball);
    double behindBallScore = position.x < world.ball.pos.x ? 0.7 : 1.0;
    double distanceFromCornerScore = calculateDistanceFromCornerScore(position, field);

    double score = 2 * closeToGoalScore + 2 * shotAtGoalScore + passLineScore + closestOpponentScore
                   + distanceFromBallScore + behindBallScore + distanceFromCornerScore;

    return score;
}
    
}
}
}
