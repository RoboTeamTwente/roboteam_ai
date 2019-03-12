//
// Created by robzelluf on 3/8/19.
//

#include <roboteam_ai/src/interface/widget.h>
#include "OffensiveCoach.h"

namespace rtt {
namespace ai {
namespace coach {

int OffensiveCoach::maxPositions = 15;
double OffensiveCoach::newRobotPositionMargin = 0.05;
double OffensiveCoach::marginFromLines = 0.2;
std::vector<OffensiveCoach::OffensivePosition> OffensiveCoach::offensivePositions;
std::map<int, OffensiveCoach::OffensivePosition> OffensiveCoach::robotPositions;

double OffensiveCoach::calculateCloseToGoalScore(Vector2 position) {
    double distanceFromGoal = (Field::get_their_goal_center() - position).length();

    double score = exp(-0.1 * distanceFromGoal);
    return score;
}

double OffensiveCoach::calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world) {
    double safelyness = 3;
    while (safelyness > 0) {
            if (control::ControlUtils::clearLine(position, Field::get_their_goal_center(), world, safelyness)) {
                break;
            }
            safelyness -= 0.5;
        }

    return 1 - exp(-0.1 * safelyness);
}


double OffensiveCoach::calculatePassLineScore(Vector2 position, roboteam_msgs::World world) {
    double safelyness = 3;
    while (safelyness > 0) {
        if (control::ControlUtils::clearLine(world.ball.pos, position, world, safelyness)) {
            break;
        }
        safelyness -= 0.5;
    }

    return 1 - exp(-0.1 * safelyness);
}

double OffensiveCoach::calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world) {
    shared_ptr<roboteam_msgs::WorldRobot> closestRobot = World::getRobotClosestToPoint(world.them, position);
    if (closestRobot) {
        double distance = (position - closestRobot->pos).length();
        return 1 - exp(-1 * distance);
    } else {
        return 1;
    }
}

double OffensiveCoach::calculatePositionScore(Vector2 position) {
    roboteam_msgs::World world = World::get_world();
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double closeToGoalScore = calculateCloseToGoalScore(position);
    double shotAtGoalScore = calculateShotAtGoalScore(position, world);
    double passLineScore = calculatePassLineScore(position, world);
    double closestOpponentScore = calculateDistanceToOpponentsScore(position, world);
    double tooCloseToBallScore = 1 - exp(-2 * (position - world.ball.pos).length());
    double behindBallScore = position.x < world.ball.pos.x ? 0.7 : 1.0;
    double distanceFromCornerScore = calculateDistanceFromCorner(position, field);

    double score = closeToGoalScore + shotAtGoalScore + passLineScore + closestOpponentScore
            + tooCloseToBallScore + behindBallScore + distanceFromCornerScore;

    return score;
}

void OffensiveCoach::calculateNewPositions() {
    std::vector<OffensiveCoach::OffensivePosition> newPositions;
    for (auto &positionPointer : offensivePositions) {
        OffensivePosition position = positionPointer;
        position.score = calculatePositionScore(position.position);

        bool tooClose = false;
        for (auto& robotPosition : robotPositions) {
            if ((robotPosition.second.position - position.position).length() < Constants::ATTACKER_DISTANCE()) {
                tooClose = true;
                break;
            }
        }
        if (!tooClose) newPositions.emplace_back(position);
    }

    offensivePositions = newPositions;

    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double xStart = 0 + marginFromLines;
    double xEnd = field.right_penalty_line.begin.x - marginFromLines;
    double yStart = -field.field_width + marginFromLines;
    double yEnd = field.field_width - marginFromLines;

    int attempts = 40;
    int attempt = 0;
    double score = 0;
    OffensivePosition position;

    while (attempt <= attempts) {
        double x = (xEnd - xStart) * ( (double)rand() / (double)RAND_MAX) + xStart;
        double y = (yEnd - yStart) * ( (double)rand() / (double)RAND_MAX) + yStart;
        Vector2 newPosition = {x, y};

        if (!Field::pointIsInField({x, y}) || Field::pointIsInDefenceArea({x, y}, false)) continue;

        bool tooClose = false;
        for (auto &robotPosition : offensivePositions) {
            if ((newPosition - robotPosition.position).length() < Constants::OFFENSIVE_POSITION_DISTANCE()) {
                tooClose = true;
                attempt++;
                break;
            }
        }

        for (auto &robotPosition : robotPositions) {
            if ((newPosition - robotPosition.second.position).length() < Constants::ATTACKER_DISTANCE()) {
                tooClose = true;
                attempt++;
                break;
            }
        }

        if (tooClose) continue;

        score = calculatePositionScore({x, y});
        position.position = {x, y};
        position.score = score;

        if (offensivePositions.size() < maxPositions) {
            position.position = {x, y};
            position.score = score;
            bool betterPosition = false;
            for (auto &positionPointer : offensivePositions) {
                if ((position.position - positionPointer.position).length() < Constants::OFFENSIVE_POSITION_DISTANCE()) {
                    if (position.score > positionPointer.score) {
                        positionPointer = position;
                        betterPosition = true;
                        attempt++;
                        break;
                    }
                }
            }

            if (!betterPosition) {
                offensivePositions.emplace_back(position);
            }
            offensivePositions.emplace_back(position);
        } else if (score > offensivePositions[maxPositions - 1].score) {
            bool betterPosition = false;
            for (auto &positionPointer : offensivePositions) {
                if ((position.position - positionPointer.position).length() < Constants::OFFENSIVE_POSITION_DISTANCE()) {
                    if (position.score > positionPointer.score) {
                        positionPointer = position;
                        betterPosition = true;
                        attempt++;
                        break;
                    }
                }
            }

            if (!betterPosition) {
                offensivePositions.emplace_back(position);
            }
        }

        attempt++;
    }

    std::sort(offensivePositions.begin(), offensivePositions.end(), compareByScore);
    offensivePositions.erase(offensivePositions.begin() + maxPositions, offensivePositions.end());
    std::cout << robotPositions.size() << std::endl;
}

bool OffensiveCoach::compareByScore(OffensivePosition position1, OffensivePosition position2) {
    return position1.score > position2.score;
}

Vector2 OffensiveCoach::calculatePositionForRobot(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {
    if ((robotPositions.find(robot->id) == robotPositions.end())) { // not there yet
        std::cout << "Calculating new position for robot " << robot->id << std::endl;
        double distance = 999;
        double currentDistance;
        OffensiveCoach::OffensivePosition newRobotPosition;
        for (auto &position : offensivePositions) {
            currentDistance = (position.position - robot->pos).length();
            if (currentDistance < distance) {
                distance = currentDistance;
                newRobotPosition = position;
            }
        }
        if (distance == 999) {
            newRobotPosition.position = robot->pos;
            newRobotPosition.score = calculatePositionScore(robot->pos);
        }

        if ((newRobotPosition.position - robot->pos).length() < 0.4) {
            robotPositions[robot->id] = newRobotPosition;
        }

        return newRobotPosition.position;

    } else {
        std::cout << "Calculating better position for robot " << robot->id << std::endl;
        calculateNewRobotPositions(robot);
        if ((robotPositions.find(robot->id) == robotPositions.end())) { //not in there
            return calculatePositionForRobot(robot);
        } else {
            return robotPositions[robot->id].position;
        }
    }
}

void OffensiveCoach::releaseRobot(int robotID) {
    robotPositions.erase(robotID);
}

Vector2 OffensiveCoach::getPositionForRobotID(int robotID) {
    Vector2 position = robotPositions[robotID].position;
    return position;
}

vector<OffensiveCoach::OffensivePosition> &OffensiveCoach::getOffensivePositions() {
    return offensivePositions;
}

double OffensiveCoach::calculateDistanceFromCorner(Vector2 position, roboteam_msgs::GeometryFieldSize field) {
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

void OffensiveCoach::calculateNewRobotPositions(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {
    int attempts = 40;
    int attempt = 0;
    OffensiveCoach::OffensivePosition currentRobotPosition = robotPositions[robot->id];

    Vector2 currentPosition = currentRobotPosition.position;
    double currentScore = calculatePositionScore(currentPosition);

    Vector2 newPosition;
    double newScore;

    bool foundNewPosition = false;

    while (attempt <= attempts) {
        double xStart = currentRobotPosition.position.x - newRobotPositionMargin;
        double xEnd = currentRobotPosition.position.x + newRobotPositionMargin;
        double yStart = currentRobotPosition.position.y - newRobotPositionMargin;
        double yEnd = currentRobotPosition.position.y + newRobotPositionMargin;

        double x = (xEnd - xStart) * ( (double)rand() / (double)RAND_MAX) + xStart;
        double y = (yEnd - yStart) * ( (double)rand() / (double)RAND_MAX) + yStart;
        newPosition = {x, y};

        bool tooClose = false;
        for (auto& robotPosition : robotPositions) {
            if (robotPosition.first != robot->id) {
                if ((newPosition - robotPosition.second.position).length() < Constants::ATTACKER_DISTANCE()) {
                    tooClose = true;
                    attempt++;
                    break;
                }
            }
        }

        if (tooClose) continue;

        if (!Field::pointIsInField(newPosition) || Field::pointIsInDefenceArea(newPosition, false)) continue;
        newScore = calculatePositionScore(newPosition);
        if (newScore > currentScore && newScore > offensivePositions[maxPositions - 1].score) {
            currentPosition = newPosition;
            currentScore = newScore;
            foundNewPosition = true;
        }
        attempt++;
    }
    if (foundNewPosition) {
        robotPositions[robot->id].position = currentPosition;
        robotPositions[robot->id].score = currentScore;
    } else {
        robotPositions.erase(robot->id);
        std::cout << "Remove old position for robot " << robot->id << std::endl;
    }
}

vector<OffensiveCoach::OffensivePosition>
OffensiveCoach::getAreaPositions(double xStart, double xEnd, double yStart, double yEnd, double numberOfPositions) {
    return vector<OffensiveCoach::OffensivePosition>();
}

vector<OffensiveCoach::OffensivePosition> OffensiveCoach::getRobotPositions() {
    std::vector<OffensivePosition> positions;
    for (auto &position : robotPositions) {
        positions.emplace_back(position.second);
    }

    return positions;
}

}
}
}