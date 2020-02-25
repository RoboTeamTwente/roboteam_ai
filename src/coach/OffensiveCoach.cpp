//
// Created by robzelluf on 3/21/19.
//

#include "coach/OffensiveCoach.h"

#include <control/ControlUtils.h>
#include <interface/api/Input.h>
#include <interface/widgets/widget.h>
#include <world/FieldComputations.h>
#include <world/World.h>

namespace rtt::ai::coach {

OffensiveCoach g_offensiveCoach;

/// Calculate new positions close to the robot in the corresponding zone.
/// In the current implementation, there are 4 zones, which are circles with radius ZONE_RADIUS in the opponent half
/// of the field. an optimal position is found within these zones using this function.
OffensiveCoach::OffensivePosition OffensiveCoach::calculateNewRobotPosition(const Field &field, const OffensivePosition &currentPosition, const Vector2 &zoneLocation, int &tick,
                                                                            Angle &targetAngle) {
    OffensivePosition bestPosition = currentPosition;
    // project the current position to the zoneLocation if it is outside
    if ((bestPosition.position - zoneLocation).length2() > ZONE_RADIUS * ZONE_RADIUS) {
        bestPosition.position = zoneLocation + (bestPosition.position - zoneLocation).stretchToLength(ZONE_RADIUS);
    }
    Angle goldenAngle = 0.01;
    tick++;
    Angle thetaPlus = tick * tick * goldenAngle + targetAngle;
    Angle thetaMinus = -1 * tick * tick * goldenAngle + targetAngle;
    std::vector<Vector2> positions = {
        bestPosition.position + thetaPlus.toVector2(1.0 * SEARCH_GRID_ROBOT_POSITIONS),  bestPosition.position + thetaPlus.toVector2(3.0 * SEARCH_GRID_ROBOT_POSITIONS),
        bestPosition.position + thetaPlus.toVector2(12.0 * SEARCH_GRID_ROBOT_POSITIONS), bestPosition.position + thetaMinus.toVector2(1.0 * SEARCH_GRID_ROBOT_POSITIONS),
        bestPosition.position + thetaMinus.toVector2(3.0 * SEARCH_GRID_ROBOT_POSITIONS), bestPosition.position + thetaMinus.toVector2(12.0 * SEARCH_GRID_ROBOT_POSITIONS)};

    auto newPosition = findBestOffensivePosition(field, positions, bestPosition, zoneLocation);
    if (newPosition.position != currentPosition.position) {
        tick = 0;
        targetAngle = newPosition.position - currentPosition.position;
    }
    return newPosition;
}

// Gets the centers of the "default locations", the 2 positions close to the goal and the 2 further away
std::vector<Vector2> OffensiveCoach::getZoneLocations(const Field &field) {
    Vector2 penaltyStretchCorner = field.getTopRightPenaltyStretch().end;
    penaltyStretchCorner.x = abs(penaltyStretchCorner.x);
    penaltyStretchCorner.y = abs(penaltyStretchCorner.y);

    std::vector<Vector2> zoneLocations;

    // Calculate two positions close to goal
    zoneLocations.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE, penaltyStretchCorner.y + CLOSE_TO_GOAL_DISTANCE);
    zoneLocations.emplace_back(penaltyStretchCorner.x - CLOSE_TO_GOAL_DISTANCE, -penaltyStretchCorner.y - CLOSE_TO_GOAL_DISTANCE);

    // Calculate two positions further from goal
    zoneLocations.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, penaltyStretchCorner.y);
    zoneLocations.emplace_back(penaltyStretchCorner.x - FURTHER_FROM_GOAL_DISTANCE, -penaltyStretchCorner.y);

    return zoneLocations;
}

void OffensiveCoach::updateOffensivePositions(const Field &field) {
    auto world = world::world->getWorld();
    std::vector<Vector2> zoneLocations = getZoneLocations(field);

    if (offensivePositions.size() != zoneLocations.size()) {
        offensivePositions = {};
        for (auto &zoneLocation : zoneLocations) {
            OffensivePosition offensivePosition;
            offensivePosition.position = zoneLocation;
            offensivePosition.score = offensiveScore.calculateOffensivePositionScore(zoneLocation, zoneLocation, world, field);
            offensivePositions.emplace_back(offensivePosition);
        }
    } else {
        static std::map<int, std::pair<int, Angle>> zoneTargets;
        for (unsigned int i = 0; i < offensivePositions.size(); i++) {
            OffensivePosition offensivePosition = offensivePositions[i];
            Vector2 zoneLocation = zoneLocations[i];
            if (zoneTargets.find(i) == zoneTargets.end()) {
                zoneTargets[i] = std::make_pair(0, Angle());
            }
            offensivePositions[i] = calculateNewRobotPosition(field, offensivePosition, zoneLocation, zoneTargets[i].first, zoneTargets[i].second);
        }
    }
}

void OffensiveCoach::addSideAttacker(const Field &field, const OffensiveCoach::RobotPtr &robot) {
    sideAttackers[robot->id] = -1;
    redistributePositions(field);
}

// TODO: Implement this function
void OffensiveCoach::addSideAttacker(const Field &field, const world_new::view::RobotView &robot) {}

void OffensiveCoach::removeSideAttacker(const OffensiveCoach::RobotPtr &robot) { sideAttackers.erase(robot->id); }

// TODO: Implement this function
void OffensiveCoach::removeSideAttacker(const world_new::view::RobotView &robot) {}

Vector2 OffensiveCoach::getPositionForRobotID(const Field &field, int robotID) {
    if (sideAttackers.find(robotID) != sideAttackers.end()) {
        int zone = sideAttackers[robotID];
        return offensivePositions[zone].position;
    } else {
        redistributePositions(field);
        return Vector2();
    }
}

void OffensiveCoach::redistributePositions(const Field &field) {
    std::unordered_map<int, Vector2> currentAttackerLocations;
    for (auto &robotIdPair : sideAttackers) {
        auto robot = world::world->getRobotForId(robotIdPair.first);
        currentAttackerLocations.insert({robot->id, robot->pos});
    }
    updateOffensivePositions(field);
    std::vector<Vector2> positions = getOffensivePositions(currentAttackerLocations.size());
    std::unordered_map<int, Vector2> shortestDistances;
    shortestDistances = rtt::Hungarian::getOptimalPairsIdentified(currentAttackerLocations, positions);
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

    int numberOfPositions = numberOfRobots + numberOfRobots % 2;

    std::vector<Vector2> positionVectors(numberOfPositions);

    for (int i = 0; i < numberOfPositions; i++) {
        positionVectors.emplace_back(offensivePositions[i].position);
    }

    return positionVectors;
}

/// this function decides what point in the goal to aim at from a position on which the ball will be/where the robot is
Vector2 OffensiveCoach::getShootAtGoalPoint(const Field &field, const Vector2 &fromPoint) {
    // get the longest line section op the visible part of the goal
    std::vector<Line> openSegments = FieldComputations::getVisiblePartsOfGoal(field, false, fromPoint, world::world->getWorld());
    if (openSegments.empty()) return field.getTheirGoalCenter();
    auto bestSegment = getLongestSegment(openSegments);

    // make two aim points which are in the corners.
    Line aimPoints = getAimPoints(field, fromPoint);
    auto leftPoint = aimPoints.start;
    auto rightPoint = aimPoints.end;

    // check if the left and right points are in the largest segment
    double maxY = std::max(bestSegment.start.y, bestSegment.end.y);
    double minY = std::min(bestSegment.start.y, bestSegment.end.y);
    bool leftPointInSegment = leftPoint.y < maxY && leftPoint.y > minY;
    bool rightPointInSegment = rightPoint.y < maxY && rightPoint.y > minY;

    // if we can aim on only one of the points, aim there, otherwise we want to aim for the centre of the largest open segment
    if (leftPointInSegment && rightPointInSegment) {
        // open goal (mostly), so just shoot in the middle of the largest open segment
        return (bestSegment.start + bestSegment.end) * 0.5;
    } else if (leftPointInSegment) {
        return leftPoint;
    } else if (rightPointInSegment) {
        return rightPoint;
    } else {
        return (bestSegment.start + bestSegment.end) * 0.5;
    }
}

Line OffensiveCoach::getAimPoints(const Field &field, const Vector2 &fromPoint) {
    Line goalSides = FieldComputations::getGoalSides(field, false);
    double angleMargin = sin(2.0 / 180.0 * M_PI);
    double constantMargin = 0.05 * field.getGoalWidth();
    Vector2 leftPoint(goalSides.start.x, goalSides.start.y + constantMargin + angleMargin * goalSides.start.dist(fromPoint));
    Vector2 rightPoint(goalSides.end.x, goalSides.end.y - angleMargin * goalSides.end.dist(fromPoint) - constantMargin);
    return Line(leftPoint, rightPoint);
}

const Line &OffensiveCoach::getLongestSegment(const std::vector<Line> &openSegments) {
    unsigned long bestIndex = 0;
    for (unsigned long i = 1; i < openSegments.size(); i++) {
        auto segment = openSegments[i];
        auto bestSegment = openSegments[bestIndex];
        if (abs(segment.start.y - segment.start.y) > abs(bestSegment.start.y - bestSegment.start.y)) {
            bestIndex = i;
        }
    }
    return openSegments[bestIndex];
}

OffensiveCoach::OffensivePosition OffensiveCoach::findBestOffensivePosition(const Field &field, const std::vector<Vector2> &positions,
                                                                            const OffensiveCoach::OffensivePosition &currentBestPosition, const Vector2 &zoneLocation) {
    // get world & field
    auto world = world::world->getWorld();

    OffensivePosition bestPosition = currentBestPosition;
    bestPosition.score = offensiveScore.calculateOffensivePositionScore(zoneLocation, bestPosition.position, world, field);

    for (auto &potentialPosition : positions) {
        // check the score and if it is better update the best position
        double potentialScore = offensiveScore.calculateOffensivePositionScore(zoneLocation, potentialPosition, world, field);
        if (potentialScore > 0.0) {
            interface::Input::drawData(interface::Visual::OFFENSE, {potentialPosition}, Qt::red, -1, interface::Drawing::DOTS, 3, 3);
        }
        if (potentialScore > bestPosition.score) {
            bestPosition = OffensivePosition(potentialPosition, potentialScore);
        }
    }

    // draw zonelocation
    interface::Input::drawData(interface::Visual::OFFENSE, {zoneLocation}, Qt::darkMagenta, -1, interface::Drawing::CIRCLES, ZONE_RADIUS * 10, ZONE_RADIUS * 10, 4);
    // draw the best point as green
    interface::Input::drawData(interface::Visual::OFFENSE, {bestPosition.position}, Qt::green, -1, interface::Drawing::DOTS, 8, 8);

    return bestPosition;
}

}  // namespace rtt::ai::coach
