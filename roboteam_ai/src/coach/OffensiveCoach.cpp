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

/// Calculate new positions close to the robot
OffensiveCoach::OffensivePosition OffensiveCoach::calculateNewRobotPosition(const OffensivePosition& currentPosition, const Vector2& defaultPosition) {
    OffensivePosition bestPosition = currentPosition;
    bestPosition.score = offensiveScore.calculateOffensivePositionScore(bestPosition.position);

    // Check all positions in a grid around the robot to look for better positions
    for (int xDiff = -GRID_SIZE; xDiff < GRID_SIZE + 1; xDiff++) {
        if (currentPosition.position.x < 0 && xDiff <= 0) continue;

        for (int yDiff = -GRID_SIZE; yDiff < GRID_SIZE + 1; yDiff++) {
            OffensivePosition newPosition;
            newPosition.position.x = currentPosition.position.x + SEARCH_GRID_ROBOT_POSITIONS * xDiff * pow(xDiff, 2);
            newPosition.position.y = currentPosition.position.y + SEARCH_GRID_ROBOT_POSITIONS * yDiff * pow(yDiff, 2);

            if (!world::field->pointIsInField(newPosition.position, 0.10)
            || world::field->pointIsInDefenceArea(newPosition.position, false)){
                continue;
            }

            if ((newPosition.position - defaultPosition).length() > ZONE_RADIUS) {
                continue;
            }

            bool tooCloseToOtherZone = false;
            for (auto& otherDefaultPosition : getDefaultLocations()) {
                if (otherDefaultPosition != defaultPosition) {
                    if ((otherDefaultPosition - newPosition.position).length() < (defaultPosition - newPosition.position).length()) {
                        tooCloseToOtherZone = true;
                        break;
                    }
                }
            }
            if (tooCloseToOtherZone) continue;
            newPosition.score = offensiveScore.calculateOffensivePositionScore(newPosition.position);

            if (newPosition.score > bestPosition.score) {
                bestPosition = newPosition;
            }
        }
    }

    return bestPosition;
}

std::vector<Vector2> OffensiveCoach::getDefaultLocations() {
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    Vector2 penaltyStretchCorner = field.top_right_penalty_stretch.end;
    penaltyStretchCorner.x = abs(penaltyStretchCorner.x);
    penaltyStretchCorner.y = abs(penaltyStretchCorner.y);

    std::vector<Vector2> defaultPositions;

    // Calculate two positions close to goal
    defaultPositions.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE, penaltyStretchCorner.y + CLOSE_TO_GOAL_DISTANCE);
    defaultPositions.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE, -penaltyStretchCorner.y - CLOSE_TO_GOAL_DISTANCE);

    // Calculate two positions further from goal
    defaultPositions.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, penaltyStretchCorner.y);
    defaultPositions.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, -penaltyStretchCorner.y);

    return defaultPositions;
}

void OffensiveCoach::updateOffensivePositions() {
    std::vector<Vector2> defaultLocations = getDefaultLocations();
    if (offensivePositions.size() != defaultLocations.size()) {
        offensivePositions = {};
        for (auto &defaultLocation : defaultLocations) {
            OffensivePosition offensivePosition;
            offensivePosition.position = defaultLocation;
            offensivePosition.score = offensiveScore.calculateOffensivePositionScore(defaultLocation);
            offensivePositions.emplace_back(offensivePosition);
        }
    } else {
        for (unsigned int i = 0; i < offensivePositions.size(); i++) {
            OffensivePosition offensivePosition = offensivePositions[i];
            Vector2 defaultPosition = defaultLocations[i];
            offensivePositions[i] = calculateNewRobotPosition(offensivePosition, defaultPosition);
        }
    }
}

void OffensiveCoach::addSideAttacker(const OffensiveCoach::RobotPtr& robot) {
    sideAttackers[robot->id] = -1;
    redistributePositions();
}

void OffensiveCoach::removeSideAttacker(const OffensiveCoach::RobotPtr& robot) {
    sideAttackers.erase(robot->id);
}

Vector2 OffensiveCoach::getPositionForRobotID(int robotID) {
    if (sideAttackers.find(robotID) != sideAttackers.end() ) {
        int zone = sideAttackers[robotID];
        return offensivePositions[zone].position;
    } else {
        redistributePositions();
        return Vector2();
    }
}

void OffensiveCoach::redistributePositions() {
    std::vector<int> robotIDs;
    for (auto &robot : sideAttackers) {
        robotIDs.emplace_back(robot.first);
    }

    updateOffensivePositions();
    std::vector<Vector2> positions = getOffensivePositions(robotIDs.size());

    rtt::HungarianAlgorithm hungarian;
    map<int, Vector2> shortestDistances;
    shortestDistances = hungarian.getRobotPositions(robotIDs, true, positions);

    for(auto &robot : sideAttackers) {
        int zone = std::find(positions.begin(), positions.end(), shortestDistances[robot.first]) - positions.begin();
        sideAttackers[robot.first] = zone;
    }
}

std::vector<Vector2> OffensiveCoach::getOffensivePositions(int numberOfRobots) {
    numberOfRobots += numberOfRobots % 2;
    std::vector<Vector2> positionVectors;

    for(int i=0; i < numberOfRobots; i++) {
        positionVectors.emplace_back(offensivePositions[i].position);
    }

    return positionVectors;
}

const set<OffensiveCoach::RobotPtr> &OffensiveCoach::getSideAttackers() const {
    return sideAttackers;
}

void OffensiveCoach::addSideAttacker(OffensiveCoach::RobotPtr robot) {
    sideAttackers.insert(robot);
}

void OffensiveCoach::removeSideAttacker(const OffensiveCoach::RobotPtr& robot) {
    int size = sideAttackers.size();
    for (auto &sideAttacker : sideAttackers) {
        if(sideAttacker->id == robot->id) {
            sideAttackers.erase(sideAttacker);
        }
    }
}

}
}
}
