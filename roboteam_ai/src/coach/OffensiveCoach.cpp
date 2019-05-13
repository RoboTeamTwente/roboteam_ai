//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/interface/widgets/widget.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "OffensiveCoach.h"

namespace rtt {
namespace ai {
namespace coach {

OffensiveCoach g_offensiveCoach;

/// Calculate new positions close to the robot
OffensiveCoach::OffensivePosition OffensiveCoach::calculateNewRobotPosition(const OffensivePosition& currentPosition, const Vector2& zoneLocation) {

    OffensivePosition bestPosition = currentPosition;
    bestPosition.score = offensiveScore.calculateOffensivePositionScore(bestPosition.position);

    // Check all positions in a grid around the robot to look for better positions
    for (int xDiff = - GRID_SIZE; xDiff < GRID_SIZE + 1; xDiff ++) {
        if (currentPosition.position.x < 0 && xDiff <= 0) continue;

        for (int yDiff = - GRID_SIZE; yDiff < GRID_SIZE + 1; yDiff ++) {
            OffensivePosition newPosition;
            newPosition.position.x = currentPosition.position.x + SEARCH_GRID_ROBOT_POSITIONS*xDiff*pow(xDiff, 2);
            newPosition.position.y = currentPosition.position.y + SEARCH_GRID_ROBOT_POSITIONS*yDiff*pow(yDiff, 2);

            if (! world::field->pointIsInField(newPosition.position, 0.10)
                    || world::field->pointIsInDefenceArea(newPosition.position, false)) {
                continue;
            }

            if ((newPosition.position - zoneLocation).length() > ZONE_RADIUS) {
                continue;
            }

            bool tooCloseToOtherZone = false;
            for (auto &otherDefaultPosition : getZoneLocations()) {
                if (otherDefaultPosition != zoneLocation) {
                    if ((otherDefaultPosition - newPosition.position).length()
                            < (zoneLocation - newPosition.position).length()) {
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

// Gets the centers of the "default locations", the 2 positions close to the goal and the 2 further away

std::vector<Vector2> OffensiveCoach::getZoneLocations() {
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    Vector2 penaltyStretchCorner = field.top_right_penalty_stretch.end;
    penaltyStretchCorner.x = abs(penaltyStretchCorner.x);
    penaltyStretchCorner.y = abs(penaltyStretchCorner.y);

    std::vector<Vector2> zoneLocations;

    // Calculate two positions close to goal
    zoneLocations.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE,
            penaltyStretchCorner.y + CLOSE_TO_GOAL_DISTANCE);
    zoneLocations.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE,
            - penaltyStretchCorner.y - CLOSE_TO_GOAL_DISTANCE);

    // Calculate two positions further from goal
    zoneLocations.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, penaltyStretchCorner.y);
    zoneLocations.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, - penaltyStretchCorner.y);

    return zoneLocations;
}

void OffensiveCoach::updateOffensivePositions() {
    std::vector<Vector2> zoneLocations = getZoneLocations();
    if (offensivePositions.size() != zoneLocations.size()) {
        offensivePositions = {};
        for (auto &zoneLocation : zoneLocations) {
            OffensivePosition offensivePosition;
            offensivePosition.position = zoneLocation;
            offensivePosition.score = offensiveScore.calculateOffensivePositionScore(zoneLocation);
            offensivePositions.emplace_back(offensivePosition);
        }
    }
    else {
        for (unsigned int i = 0; i < offensivePositions.size(); i++) {
            OffensivePosition offensivePosition = offensivePositions[i];
            Vector2 zoneLocation = zoneLocations[i];
            offensivePositions[i] = calculateNewRobotPosition(offensivePosition, zoneLocation);
        }
    }
}

void OffensiveCoach::addSideAttacker(const OffensiveCoach::RobotPtr &robot) {
    sideAttackers[robot->id] = - 1;
    redistributePositions();
}

void OffensiveCoach::removeSideAttacker(const OffensiveCoach::RobotPtr &robot) {
    sideAttackers.erase(robot->id);
}

Vector2 OffensiveCoach::getPositionForRobotID(int robotID) {
    if (sideAttackers.find(robotID) != sideAttackers.end()) {
        int zone = sideAttackers[robotID];
        return offensivePositions[zone].position;
    }
    else {
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

    for (auto &robot : sideAttackers) {
        int zone = std::find(positions.begin(), positions.end(), shortestDistances[robot.first]) - positions.begin();
        sideAttackers[robot.first] = zone;
    }
}

std::vector<Vector2> OffensiveCoach::getOffensivePositions(int numberOfRobots) {
    // The offensive positions are symmetric, meaning that there's a position close to the goal, both left and right,
    // and a position a bit further, left and right. If you have 1 sideAttacker, you want that robot to be able to choose
    // between the two close positions. If you have 3, you want them to choose from all 4. Hence, the number of positions
    // is rounded up to a multiple of 2.

    int numberOfPositions = numberOfRobots + numberOfRobots%2;

    std::vector<Vector2> positionVectors;

    for (int i = 0; i < numberOfPositions; i ++) {
        positionVectors.emplace_back(offensivePositions[i].position);
    }

    return positionVectors;
}

/// this function decides what point in the goal to aim at from a position on which the ball will be/where the robot is
Vector2 OffensiveCoach::getShootAtGoalPoint(const Vector2 &fromPoint) {
    std::vector<std::pair<Vector2, Vector2>> openSegments = world::field->getVisiblePartsOfGoal(false, fromPoint);
    if (! openSegments.empty()) {
        // sort on size
        std::sort(openSegments.begin(), openSegments.end(),
                [](std::pair<Vector2, Vector2> a, std::pair<Vector2, Vector2> b) {
                  return abs(a.first.y - a.second.y) > abs(b.first.y - b.second.y);
                });
        double maxY = max(openSegments[0].first.y, openSegments[0].second.y);
        double minY = min(openSegments[0].first.y, openSegments[0].second.y);

        // make two aim points which are in the corners.
        std::pair<Vector2, Vector2> goalSides = world::field->getGoalSides(false);
        double angleMargin = sin(2.0/180.0*M_PI);
        double constantMargin=0.05*world::field->get_field().goal_width;
        Vector2 leftPoint(goalSides.first.x, goalSides.first.y + constantMargin+angleMargin*goalSides.first.dist(fromPoint));
        Vector2 rightPoint(goalSides.second.x, goalSides.second.y - angleMargin*goalSides.second.dist(fromPoint)-constantMargin);

        bool leftPointInSegment = leftPoint.y <= maxY && leftPoint.y >= minY;
        bool rightPointInSegment = rightPoint.y <= maxY && rightPoint.y >= minY;
        // if we can aim on only one of the points, aim there, otherwise we want to aim for the centre of the largest open segment
        if (leftPointInSegment && rightPointInSegment) {
            // open goal (mostly), so just shoot in the middle of the largest open segment
            return (openSegments[0].first + openSegments[0].second)*0.5;
        }
        else if (leftPointInSegment) {
            return leftPoint;
        }
        else if (rightPointInSegment) {
            return rightPoint;
        }
        else {
            return (openSegments[0].first + openSegments[0].second)*0.5;
        }
    }
    return world::field->get_their_goal_center();

}
}
}
}
