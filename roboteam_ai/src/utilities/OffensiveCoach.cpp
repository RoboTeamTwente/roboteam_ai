//
// Created by robzelluf on 3/8/19.
//

#include <roboteam_ai/src/interface/widget.h>
#include "OffensiveCoach.h"

namespace rtt{
namespace ai{
namespace coach {

int OffensiveCoach::maxPositions = 20;
std::vector<OffensiveCoach::offensivePosition> OffensiveCoach::offensivePositions;
std::map<int, int> OffensiveCoach::robotPositions;

double OffensiveCoach::calculateCloseToGoalScore(Vector2 position) {
    double distanceFromGoal = (Field::get_their_goal_center() - position).length();

    double score = exp(-0.05 * distanceFromGoal);
    return score;
}

double OffensiveCoach::calculateShotAtGoalScore(Vector2 position, roboteam_msgs::World world) {
    double safelyness = 3;
    while (safelyness > 0) {
            if (control::ControlUtils::clearLine(position, Field::get_their_goal_center(), world, safelyness)) {
                return safelyness / 3;
            }
            safelyness -= 0.5;
        }

    return 0;
}


double OffensiveCoach::calculatePassLineScore(Vector2 position, roboteam_msgs::World world) {
    double safelyness = 3;
    while (safelyness > 0) {
        if (control::ControlUtils::clearLine(world.ball.pos, position, world, safelyness)) {
            return safelyness / 3;
        }
        safelyness -= 0.5;
    }

    return 0;
}

double OffensiveCoach::calculateDistanceToOpponentsScore(Vector2 position, roboteam_msgs::World world) {
    shared_ptr<roboteam_msgs::WorldRobot> closestRobot = World::getRobotClosestToPoint(world.them, position);
    if (closestRobot) {
        double distance = (position - closestRobot->pos).length();
        return 1 - exp(-0.01 * distance);
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
    double tooCloseToBallScore = 1 - exp(-5 * (position - world.ball.pos).length());
    double behindBallScore = position.x < world.ball.pos.x ? 0.7 : 1.0;

    double score = closeToGoalScore + shotAtGoalScore + passLineScore + closestOpponentScore
            + tooCloseToBallScore + behindBallScore;

    return score;
}

void OffensiveCoach::calculateNewPositions() {
    for (auto &positionPointer : offensivePositions) {
        offensivePosition position = positionPointer;
        position.score = calculatePositionScore(position.position);
        positionPointer = position;
    }

    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double margin = 0.2;
    double xStart = 0 + margin;
    double xEnd = field.field_length / 2 - margin;
    double yStart = -field.field_width + margin;
    double yEnd = field.field_width - margin;

    int attempts = 20;
    int attempt = 0;
    double score = 0;
    offensivePosition position;

    while (attempt <= attempts) {
        double x = (xEnd - xStart) * ( (double)rand() / (double)RAND_MAX) + xStart;
        double y = (yEnd - yStart) * ( (double)rand() / (double)RAND_MAX) + yStart;

        if (!Field::pointIsInField({x, y}) || Field::pointIsInDefenceArea({x, y}, false)) continue;

        score = calculatePositionScore({x, y});
        position.position = {x, y};
        position.score = score;

        if (offensivePositions.size() < maxPositions) {
            position.position = {x, y};
            position.score = score;
            bool betterPosition = false;
            for (auto &positionPointer : offensivePositions) {
                if ((position.position - positionPointer.position).length() < Constants::ATTACKERS_DISTANCE()) {
                    if (position.score > positionPointer.score) {
                        positionPointer = position;
                        betterPosition = true;
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
                if ((position.position - positionPointer.position).length() < Constants::ATTACKERS_DISTANCE()) {
                    if (position.score > positionPointer.score) {
                        positionPointer = position;
                        betterPosition = true;
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
    std::cout << offensivePositions[0].score << " - " << offensivePositions[maxPositions - 1].score << std::endl;
}

bool OffensiveCoach::compareByScore(const offensivePosition position1, const offensivePosition position2) {
    return position1.score > position2.score;
}

void OffensiveCoach::setRobot(int robotID) {
    for (offensivePosition position : offensivePositions) {
        std::cout << position.position << std::endl;
    }

    std::map<int,int>::iterator it;
    int i = 0;
    while (i <= maxPositions) {
        for (it = robotPositions.begin(); it != robotPositions.end(); ++it) {
            if (it->second == i) {
                continue;
            }
            robotPositions[robotID] = i;
        }
        i++;
    }
}

void OffensiveCoach::releaseRobot(int robotID) {
    robotPositions.erase(robotID);
}

Vector2 OffensiveCoach::getPositionForRobotID(int robotID) {
    int positionIndex = robotPositions[robotID];
    Vector2 position = offensivePositions[positionIndex].position;
    std::cout << "Robot " << robotID << " - " << position << std::endl;
    return position;
}

const vector<OffensiveCoach::offensivePosition> &OffensiveCoach::getOffensivePositions() {
    return offensivePositions;
}

}
}
}