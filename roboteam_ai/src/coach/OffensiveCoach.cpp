//
// Created by robzelluf on 3/21/19.
//

#include <roboteam_ai/src/interface/widgets/widget.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "OffensiveCoach.h"
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/world/Field.h>

namespace rtt {
namespace ai {
namespace coach {

OffensiveCoach g_offensiveCoach;

/// Calculate new positions close to the robot in the corresponding zone.
/// In the current implementation, there are 4 zones, which are circles with radius ZONE_RADIUS in the opponent half
/// of the field. an optimal position is found within these zones using this function.
OffensiveCoach::OffensivePosition OffensiveCoach::calculateNewRobotPosition(const OffensivePosition &currentPosition,
        const Vector2 &zoneLocation, int &tick, Angle &targetAngle) {

    OffensivePosition bestPosition = currentPosition;
    // project the current position to the zoneLocation if it is outside
    if ((bestPosition.position - zoneLocation).length2() > ZONE_RADIUS*ZONE_RADIUS) {
        bestPosition.position = zoneLocation + (bestPosition.position - zoneLocation).stretchToLength(ZONE_RADIUS);
    }
    Angle goldenAngle = 0.01;//2.399963;
    tick++;
    Angle thetaPlus = tick*tick*goldenAngle + targetAngle;
    Angle thetaMinus = -1*tick*tick*goldenAngle + targetAngle;
    std::vector<Vector2> positions = {bestPosition.position + thetaPlus.toVector2(1.0   * SEARCH_GRID_ROBOT_POSITIONS),
                                      bestPosition.position + thetaPlus.toVector2(3.0   * SEARCH_GRID_ROBOT_POSITIONS),
                                      bestPosition.position + thetaPlus.toVector2(12.0  * SEARCH_GRID_ROBOT_POSITIONS),
                                      bestPosition.position + thetaMinus.toVector2(1.0  * SEARCH_GRID_ROBOT_POSITIONS),
                                      bestPosition.position + thetaMinus.toVector2(3.0  * SEARCH_GRID_ROBOT_POSITIONS),
                                      bestPosition.position + thetaMinus.toVector2(12.0 * SEARCH_GRID_ROBOT_POSITIONS)};

    auto newPosition = findBestOffensivePosition(positions, bestPosition, zoneLocation);
    if (newPosition.position != currentPosition.position) {
        tick = 0;
        targetAngle = newPosition.position - currentPosition.position;
    }
    return newPosition;
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

    auto world = world::world->getWorld();
    auto field = world::field->get_field();

    std::vector<Vector2> zoneLocations = getZoneLocations();
    if (offensivePositions.size() != zoneLocations.size()) {
        offensivePositions = {};
        for (auto &zoneLocation : zoneLocations) {
            OffensivePosition offensivePosition;
            offensivePosition.position = zoneLocation;
            offensivePosition.score = offensiveScore.calculateOffensivePositionScore(zoneLocation, zoneLocation, world, field);
            offensivePositions.emplace_back(offensivePosition);
        }
    }
    else {
        static std::map<int, std::pair<int, Angle>> zoneTargets;
        for (unsigned int i = 0; i < offensivePositions.size(); i ++) {
            OffensivePosition offensivePosition = offensivePositions[i];
            Vector2 zoneLocation = zoneLocations[i];
            if (zoneTargets.find(i) == zoneTargets.end()) {
                zoneTargets[i] = std::make_pair(0, Angle());
            }
            offensivePositions[i] = calculateNewRobotPosition(offensivePosition, zoneLocation,
                    zoneTargets[i].first, zoneTargets[i].second);
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

    // get the longest line section op the visible part of the goal
    std::vector<std::pair<Vector2, Vector2>> openSegments = world::field->getVisiblePartsOfGoal(false, fromPoint);
    if (openSegments.empty()) return world::field->get_their_goal_center();
    auto bestSegment = getLongestSegment(openSegments);

    // make two aim points which are in the corners.
    std::pair<Vector2, Vector2> aimPoints = getAimPoints(fromPoint);
    auto leftPoint = aimPoints.first;
    auto rightPoint = aimPoints.second;

    // check if the left and right points are in the largest segment
    double maxY = std::max(bestSegment.first.y, bestSegment.second.y);
    double minY = std::min(bestSegment.first.y, bestSegment.second.y);
    bool leftPointInSegment = leftPoint.y < maxY && leftPoint.y > minY;
    bool rightPointInSegment = rightPoint.y < maxY && rightPoint.y > minY;

    // if we can aim on only one of the points, aim there, otherwise we want to aim for the centre of the largest open segment
    if (leftPointInSegment && rightPointInSegment) {
        // open goal (mostly), so just shoot in the middle of the largest open segment
        return (bestSegment.first + bestSegment.second)*0.5;
    }
    else if (leftPointInSegment) {
        return leftPoint;
    }
    else if (rightPointInSegment) {
        return rightPoint;
    }
    else {
        return (bestSegment.first + bestSegment.second)*0.5;
    }

}

std::pair<Vector2, Vector2> OffensiveCoach::getAimPoints(const Vector2 &fromPoint) {
    std::pair<Vector2, Vector2> goalSides = world::field->getGoalSides(false);
    double angleMargin = sin(2.0/180.0*M_PI);
    double constantMargin = 0.05*world::field->get_field().goal_width;
    Vector2 leftPoint(goalSides.first.x,
            goalSides.first.y + constantMargin + angleMargin*goalSides.first.dist(fromPoint));
    Vector2 rightPoint(goalSides.second.x,
            goalSides.second.y - angleMargin*goalSides.second.dist(fromPoint) - constantMargin);
    return std::make_pair(leftPoint, rightPoint);
}

const std::pair<Vector2, Vector2> &OffensiveCoach::getLongestSegment(
        const std::vector<std::pair<Vector2, Vector2>> &openSegments) {

    unsigned long bestIndex = 0;
    for (unsigned long i = 1; i < openSegments.size(); i ++) {
        auto segment = openSegments[i];
        auto bestSegment = openSegments[bestIndex];
        if (abs(segment.first.y - segment.second.y) > abs(bestSegment.first.y - bestSegment.second.y)) {
            bestIndex = i;
        }
    }
    return openSegments[bestIndex];
}

OffensiveCoach::OffensivePosition OffensiveCoach::findBestOffensivePosition(const std::vector<Vector2> &positions,
        const OffensiveCoach::OffensivePosition &currentBestPosition, const Vector2 &zoneLocation) {

    // get world & field
    auto world = world::world->getWorld();
    auto field = world::field->get_field();

    OffensivePosition bestPosition = currentBestPosition;
    bestPosition.score =
            offensiveScore.calculateOffensivePositionScore(zoneLocation, bestPosition.position, world, field);

    for (auto &potentialPosition : positions) {
        // check the score and if it is better update the best position
        double potentialScore =
                offensiveScore.calculateOffensivePositionScore(zoneLocation, potentialPosition, world, field);
        if (potentialScore > 0.0) {
            interface::Input::drawData(interface::Visual::OFFENSE, {potentialPosition}, Qt::red, - 1,
                    interface::Drawing::DOTS, 3, 3);
        }
        if (potentialScore > bestPosition.score) {
            bestPosition = OffensivePosition(potentialPosition, potentialScore);
        }
    }

    // draw zonelocation
    interface::Input::drawData(interface::Visual::OFFENSE, {zoneLocation}, Qt::darkMagenta, - 1,
            interface::Drawing::CIRCLES, ZONE_RADIUS*10, ZONE_RADIUS*10, 4);
    // draw the best point as green
    interface::Input::drawData(interface::Visual::OFFENSE, {bestPosition.position}, Qt::green, - 1,
            interface::Drawing::DOTS, 8, 8);

    return bestPosition;
}


}
}
}
