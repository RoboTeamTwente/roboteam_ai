//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/interface/widget.h>
#include <roboteam_ai/src/interface/drawer.h>
#include "OffensiveCoach.h"

namespace rtt {
namespace ai {
namespace coach {

OffensiveCoach g_offensiveCoach;

/// Recalculate the score of each offensive position, and remove it if it is too close to any of the robotPositions
void OffensiveCoach::recalculateOffensivePositions() {
    auto it = offensivePositions.begin();
    while (it < offensivePositions.end()) {
        it->score = CoachHeuristics::calculatePositionScore(it->position);
        for (auto& robotPosition : robotPositions) {
            if ((it->position - robotPosition.second.position).length() < ATTACKER_DISTANCE) {
                offensivePositions.erase(it);
                break;
            }
        }
        it++;
    }
}

/// Calculate a random position within margins
OffensiveCoach::OffensivePosition OffensiveCoach::calculateRandomPosition(double xStart, double xEnd, double yStart,
                                                                          double yEnd) {
    OffensivePosition position;

    double x = (xEnd - xStart) * ( (double)rand() / (double)RAND_MAX) + xStart;
    double y = (yEnd - yStart) * ( (double)rand() / (double)RAND_MAX) + yStart;

    position.position = {x, y};

    if (!Field::pointIsInField(position.position) || Field::pointIsInDefenceArea(position.position, false)) {
        return calculateRandomPosition(xStart, xEnd, yStart, yEnd);
    }

    return position;
}

/// Check if position is too close to any of the robot positions
bool OffensiveCoach::positionTooCloseToRobotPositions(OffensivePosition position, int self) {
    bool tooClose = false;
    for (auto &robotPosition : robotPositions) {
        if (robotPosition.first != self) {
            if ((position.position - robotPosition.second.position).length() < ATTACKER_DISTANCE) {
                tooClose = true;
                continue;
            }
        }
    }

    return tooClose;
}

/// Calculate an x amount of new offensive positions and compare them with the old ones
void OffensiveCoach::calculateNewPositions() {
    recalculateOffensivePositions();

    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double xStart = 0 + marginFromLines;
    double xEnd = field.right_penalty_line.begin.x - marginFromLines;
    double yStart = -field.field_width + marginFromLines;
    double yEnd = field.field_width - marginFromLines;

    int attempts = 30;

    for (int attempt = 0; attempt < attempts; attempt++) {
        OffensivePosition position = calculateRandomPosition(xStart, xEnd, yStart, yEnd);
        if (positionTooCloseToRobotPositions(position)) continue;
        position.score =  CoachHeuristics::calculatePositionScore(position.position);

        if (offensivePositions.empty()) {
            offensivePositions.push_back(position);
            continue;
        }

        compareToCurrentPositions(position);
    }

    std::sort(offensivePositions.begin(), offensivePositions.end(), compareByScore);
    if (offensivePositions.size() > maxPositions) {
        offensivePositions.erase(offensivePositions.begin() + maxPositions, offensivePositions.end());
    }
    drawOffensivePoints();
}

/// Compare scores to each other
void OffensiveCoach::compareToCurrentPositions(const OffensiveCoach::OffensivePosition &position) {
    OffensivePosition bestPos = position;
    auto it = offensivePositions.begin();
    while (it < offensivePositions.end()) {
        if ((position.position - it->position).length() < OFFENSIVE_POSITION_DISTANCE) {//Constants::ATTACKER_DISTANCE()) {
            if (it->score >= bestPos.score) {
                bestPos.position = it->position;
                bestPos.score = it->score;
            }
            offensivePositions.erase(it);
        }
        it++;
    }
    offensivePositions.push_back(bestPos);
}

bool OffensiveCoach::compareByScore(OffensivePosition position1, OffensivePosition position2) {
    return position1.score > position2.score;
}

/// Get new position for robot, or recalculate it's old one
Vector2 OffensiveCoach::calculatePositionForRobot(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {
    if ((robotPositions.find(robot->id) == robotPositions.end())) { // not there yet
        return getClosestOffensivePosition(robot);

    } else {
        calculateNewRobotPositions(robot);
        if ((robotPositions.find(robot->id) == robotPositions.end())) { //not in there
            return calculatePositionForRobot(robot);
        } else {
            return robotPositions[robot->id].position;
        }
    }
}

/// Get closest offensive position to robot
Vector2 OffensiveCoach::getClosestOffensivePosition(const shared_ptr<roboteam_msgs::WorldRobot> &robot) {
    double distance = INT_MAX;
    double currentDistance;
    OffensivePosition newRobotPosition;
    for (auto &position : offensivePositions) {
        currentDistance = (position.position - robot->pos).length();
        if (currentDistance < distance) {
            distance = currentDistance;
            newRobotPosition = position;
        }
    }
    if (distance == INT_MAX) {
        newRobotPosition.position = robot->pos;
        newRobotPosition.score = CoachHeuristics::calculatePositionScore(robot->pos);
    }


    if ((newRobotPosition.position - robot->pos).length() < 0.8) {
        robotPositions[robot->id] = newRobotPosition;
    }
    return newRobotPosition.position;
}

void OffensiveCoach::releaseRobot(int robotID) {
    robotPositions.erase(robotID);
}

/// Getter for robot position
Vector2 OffensiveCoach::getPositionForRobotID(int robotID) {
    Vector2 position = robotPositions[robotID].position;
    return position;
}

/// Calculate new positions close to the robot
void OffensiveCoach::calculateNewRobotPositions(std::shared_ptr<roboteam_msgs::WorldRobot> robot) {
    OffensiveCoach::OffensivePosition currentRobotPosition = robotPositions[robot->id];

    Vector2 currentPosition = currentRobotPosition.position;
    double currentScore = CoachHeuristics::calculatePositionScore(currentPosition);

    OffensivePosition newPosition;
    bool foundNewPosition = false;

    for (int xDiff : {-2, -1, 0, 1, 2}) {
        for (int yDiff : {-2, -1, 0, 1, 2}) {
            newPosition.position.x = currentPosition.x + 0.01 * xDiff;
            newPosition.position.y = currentPosition.y + 0.01 * yDiff;
            if (!Field::pointIsInField(newPosition.position) || Field::pointIsInDefenceArea(newPosition.position, false)) continue;

            if(positionTooCloseToRobotPositions(newPosition, robot->id)) continue;

            newPosition.score = CoachHeuristics::calculatePositionScore(newPosition.position);
            if (newPosition.score > currentScore && newPosition.score > 0.6 * offensivePositions[maxPositions - 1].score) {
                currentPosition = newPosition.position;
                currentScore = newPosition.score;
                foundNewPosition = true;
            }

        }
    }
    if (foundNewPosition) {
        robotPositions[robot->id].position = currentPosition;
        robotPositions[robot->id].score = currentScore;
    } else {
        robotPositions.erase(robot->id);

    }
}

vector<OffensiveCoach::OffensivePosition> OffensiveCoach::getRobotPositionVectors() {
    std::vector<OffensivePosition> positions;
    for (auto &position : robotPositions) {
        positions.emplace_back(position.second);
    }

    return positions;
}

/// Set offensive positions to be drawn
void OffensiveCoach::drawOffensivePoints() {
    /// Draw general offensive points
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    for (auto &offensivePosition : offensivePositions) {
        displayColorData.emplace_back(std::make_pair(offensivePosition.position, Qt::green));
    }
    interface::Drawer::setOffensivePoints(displayColorData);

    /// Draw attacker points specific to robot
    displayColorData = {};
    for (auto robotPosition : robotPositions) {
        displayColorData.emplace_back(std::make_pair(robotPosition.second.position, Qt::darkYellow));
        interface::Drawer::setAttackerPoints(robotPosition.first, displayColorData);
    }

}

/// Get robot with highest offensive score
int OffensiveCoach::getBestStrikerID() {
    int bestRobot = -1;
    double bestScore = 0;
    for (auto& robotPosition : robotPositions) {
        if (robotPosition.second.score > bestScore) {
            bestRobot = robotPosition.first;
            bestScore = robotPosition.second.score;
        }
    }

    if (bestRobot == -1) {
        for (auto& robot : World::get_world().us) {
            double score = CoachHeuristics::calculatePositionScore(robot.pos);
            if (score > bestScore) {
                bestRobot = robot.id;
                bestScore = score;
            }
        }
    }

    return bestRobot;
}

const vector<OffensiveCoach::OffensivePosition> &OffensiveCoach::getOffensivePositions() {
    return offensivePositions;
}

const map<int, OffensiveCoach::OffensivePosition> &OffensiveCoach::getRobotPositions() {
    return robotPositions;
}

}
}
}
