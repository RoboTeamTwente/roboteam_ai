//
// Created by maxl on 11-02-21.
//

#include "stp/computations/GoalComputations.h"

#include <roboteam_utils/Grid.h>

#include "world/Field.h"
#include "world/World.hpp"

namespace rtt::ai::stp::computations {
Vector2 GoalComputations::calculateGoalTarget(rtt_world::World *world, const rtt_world::Field &field) {
    // Position of the ball from which the goal target is determined
    auto sourcePoint = world->getWorld().value().getBall().value()->position;

    // Make a vector with all robotViews to see if one of the robots is in front of the goal
    std::vector<world::view::RobotView> allRobotsVec = world->getWorld().value().getUs();
    std::vector<world::view::RobotView> allRobotsVec2 = world->getWorld().value().getThem();
    std::copy(allRobotsVec2.begin(), allRobotsVec2.end(), std::back_inserter(allRobotsVec));
    // Get the longest line section on the visible part of the goal
    std::vector<LineSegment> openSegments = FieldComputations::getVisiblePartsOfGoal(field, false, sourcePoint, allRobotsVec);

    // If there is no empty location to shoot at, just shoot at the center of the goal
    /// TODO-Max communicate this to the play
    if (openSegments.empty()) return field.getTheirGoalCenter();

    // The longest open segment of the goal will be the best to shoot at
    LineSegment bestSegment = getLongestSegment(openSegments);

    // Make two aim points which are in the corners, since these points are harder for the keeper to intercept
    LineSegment aimPoints = getAimPoints(field, sourcePoint);
    auto leftPoint = aimPoints.start;
    auto rightPoint = aimPoints.end;

    // Check if the left and right points are in the best segment
    double maxY = std::max(bestSegment.start.y, bestSegment.end.y);
    double minY = std::min(bestSegment.start.y, bestSegment.end.y);
    bool leftPointInSegment = leftPoint.y < maxY && leftPoint.y > minY;
    bool rightPointInSegment = rightPoint.y < maxY && rightPoint.y > minY;

    // If we can aim on only one of the points, aim there, otherwise we want to aim for the centre of the largest open segment
    if (leftPointInSegment && rightPointInSegment) {
        // Open goal (mostly), so just shoot in the middle of the largest open segment
        return (bestSegment.start + bestSegment.end) * 0.5;
    } else if (leftPointInSegment) {
        return leftPoint;
    } else if (rightPointInSegment) {
        return rightPoint;
    } else {
        return (bestSegment.start + bestSegment.end) * 0.5;
    }
}

LineSegment GoalComputations::getAimPoints(const world::Field &field, const Vector2 &sourcePoint) {
    LineSegment goalSides = FieldComputations::getGoalSides(field, false);

    // Aim points are located some distance away from the edges of the goal to take into account inaccuracies in the shot
    const double angleMargin = sin(2.0 / 180.0 * M_PI);
    const double constantMargin = 0.05 * field.getGoalWidth();
    Vector2 leftPoint(goalSides.start.x, goalSides.start.y + constantMargin + angleMargin * goalSides.start.dist(sourcePoint));
    Vector2 rightPoint(goalSides.end.x, goalSides.end.y - angleMargin * goalSides.end.dist(sourcePoint) - constantMargin);

    return LineSegment(leftPoint, rightPoint);
}

const LineSegment &GoalComputations::getLongestSegment(const std::vector<LineSegment> &openSegments) {
    return *std::max_element(openSegments.begin(), openSegments.end(),
                             [](const LineSegment &left, const LineSegment &right) { return fabs(left.start.y - left.end.y) < fabs(right.start.y - right.end.y); });
}
}  // namespace rtt::ai::stp::computations
