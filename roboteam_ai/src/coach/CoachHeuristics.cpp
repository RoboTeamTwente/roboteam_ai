//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/analysis/GameAnalyzer.h>
#include "CoachHeuristics.h"

namespace rtt {
namespace ai {
namespace coach {

const double CoachHeuristics::MAX_DISTANCE_FROM_BALL = 6.0;
const double CoachHeuristics::CLOSE_TO_GOAL_WEIGHT = -0.1;
const double CoachHeuristics::SHOT_AT_GOAL_WEIGHT = -3.0;
const double CoachHeuristics::PASS_LINE_WEIGHT = -3.0;
const double CoachHeuristics::DISTANCE_TO_OPPONENTS_WEIGHT = -3;
const double CoachHeuristics::DISTANCE_FROM_CORNER_WEIGHT = -0.05;
const double CoachHeuristics::BEHIND_BALL_WEIGHT = -0.1;

/// Gives a higher score to positions closer to the oppontents goal
double CoachHeuristics::calculateCloseToGoalScore(const Vector2 &position) {
    double distanceFromGoal = (world::field->get_their_goal_center() - position).length();

    double score = exp(CLOSE_TO_GOAL_WEIGHT * distanceFromGoal);
    return score;
}

/// Gives a higher score if the line between the position and the goal is free
double CoachHeuristics::calculateShotAtGoalScore(const Vector2& position, WorldData world) {
    double viewAtGoal = world::field->getPercentageOfGoalVisibleFromPoint(false, position, world) / 100;
    return 1 - exp(SHOT_AT_GOAL_WEIGHT * viewAtGoal);

    }

/// Gives a higher score if the distance between the ball and the positions if free (safe pass line)
double CoachHeuristics::calculatePassLineScore(const Vector2& position, WorldData world) {
    double smallestAngle = M_PI / 4;
    auto ball = world.ball;

    for(const auto& robot : world.them) {
        if(control::ControlUtils::isPointProjectionOnLine(robot.pos, ball.pos, position)) {
            double angle = abs((position - ball.pos).toAngle() - (robot.pos - ball.pos).toAngle());
            if (angle < smallestAngle) {
                smallestAngle = angle;
            }
        }
    }

    return 1 - exp(PASS_LINE_WEIGHT * smallestAngle);
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

/// Calculates a total score based on all the sub-scores
double CoachHeuristics::calculateOffensivePositionScore(const Vector2 &position) {
    WorldData world = world::world->getWorld();
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();

    double closeToGoalScore = calculateCloseToGoalScore(position);
    double passLineScore = calculatePassLineScore(position, world);
    double shotAtGoalScore = calculateShotAtGoalScore(position, world);
    double distanceToOpponentScore = calculateDistanceToOpponentsScore(position, world);

    double score = shotAtGoalScore + passLineScore + closeToGoalScore + distanceToOpponentScore;
    return score;
}

double CoachHeuristics::calculatePassScore(const Vector2 &position) {
    WorldData world = world::world->getWorld();
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    double closeToGoalScore = calculateCloseToGoalScore(position);
    double shotAtGoalScore = calculateShotAtGoalScore(position, world);
    double passLineScore = calculatePassLineScore(position, world);
    double behindBallScore = calculateBehindBallScore(position, world);
    double distanceToOpponentScore = calculateDistanceToOpponentsScore(position, world);

    double score = closeToGoalScore + shotAtGoalScore + 2 * passLineScore + behindBallScore + distanceToOpponentScore;
    return score;
}

double CoachHeuristics::calculateBehindBallScore(const Vector2 &position, CoachHeuristics::WorldData world) {
    double xDistanceBehindBall = world.ball.pos.x - position.x;
    if (xDistanceBehindBall < 0) {
        return 1.0;
    } else {
        return exp(BEHIND_BALL_WEIGHT * xDistanceBehindBall);
    }
}

}
}
}
