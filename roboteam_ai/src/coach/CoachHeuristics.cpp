//
// Created by robzelluf on 3/21/19.
//

#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

double CoachHeuristics::maxDistanceFromBall = 6.0;

double CoachHeuristics::calculateCloseToGoalScore(Vector2 position) {
    double distanceFromGoal = (Field::get_their_goal_center() - position).length();

    double score = exp(-0.1 * distanceFromGoal);
    return score;
}

double CoachHeuristics::calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world) {
    double safeDistanceFactor = 3;
    while (safeDistanceFactor > 0) {
        if (control::ControlUtils::clearLine(position, Field::get_their_goal_center(), world, safeDistanceFactor)) {
            break;
        }
        safeDistanceFactor -= 0.5;
    }

    return 1 - exp(-safeDistanceFactor);
}


double CoachHeuristics::calculatePassLineScore(Vector2 position, roboteam_msgs::World world) {
    double safeDistanceFactor = 3;
    while (safeDistanceFactor > 0) {
        if (control::ControlUtils::clearLine(world.ball.pos, position, world, safeDistanceFactor)) {
            break;
        }
        safeDistanceFactor -= 0.5;
    }

    return 1 - exp(-safeDistanceFactor);
}

double CoachHeuristics::calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world) {
    shared_ptr<roboteam_msgs::WorldRobot> closestRobot = World::getRobotClosestToPoint(world.them, position);
    if (closestRobot) {
        double distance = (position - closestRobot->pos).length();
        return 1 - exp(-0.5 * distance);
    } else {
        return 1;
    }
}

double CoachHeuristics::calculateDistanceFromCornerScore(Vector2 position, roboteam_msgs::GeometryFieldSize field) {
    Vector2 corner;
    corner.x = field.field_length / 2;
    if (position.y > 0) {
        corner.y = field.field_width / 2;
    } else {
        corner.y = -field.field_width / 2;
    }
    double distanceFromCorner = (position - corner).length();
    return 1 - exp(-0.05 * distanceFromCorner);
}

double CoachHeuristics::calculateDistanceFromBallScore(Vector2 position, roboteam_msgs::GeometryFieldSize& field, roboteam_msgs::WorldBall& ball) {
    double distanceFromBall = (position - ball.pos).length();
    return -pow(distanceFromBall / (0.5 * maxDistanceFromBall), 2) + 2 * (distanceFromBall / (0.5 * maxDistanceFromBall));
}

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
