#include "world/FieldComputations.h"

#include <roboteam_utils/Shadow.h>

#include "world/views/WorldDataView.hpp"

namespace rtt::ai {

bool FieldComputations::pointIsInOurDefenseArea(const rtt_world::Field &field, const Vector2 &point, double margin, double backMargin) {
    auto defenseArea = FieldComputations::getDefenseArea(field, true, margin, backMargin);
    return defenseArea.contains(point);
}

bool FieldComputations::pointIsInTheirDefenseArea(const rtt_world::Field &field, const Vector2 &point, double margin, double backMargin) {
    auto defenseArea = FieldComputations::getDefenseArea(field, false, margin, backMargin);
    return defenseArea.contains(point);
}

bool FieldComputations::pointIsInField(const rtt_world::Field &field, const Vector2 &point, double margin) {
    return (point.x <= field.getRightmostX() + margin && point.x >= field.getLeftmostX() - margin && point.y <= field.getTopmostY() + margin &&
            point.y >= field.getBottommostY() - margin);
}

bool FieldComputations::pointIsValidPosition(const rtt_world::Field &field, const Vector2 &point, stp::AvoidObjects avoidObjects, double fieldMargin, double ourDefenseAreaMargin,
                                             double theirDefenseAreaMargin) {
    if (avoidObjects.shouldAvoidOutOfField && !pointIsInField(field, point, fieldMargin)) return false;
    if (avoidObjects.shouldAvoidDefenseArea && (pointIsInOurDefenseArea(field, point, ourDefenseAreaMargin) || pointIsInTheirDefenseArea(field, point, theirDefenseAreaMargin)))
        return false;
    return true;
}

double FieldComputations::getTotalGoalAngle(const rtt_world::Field &field, bool ourGoal, const Vector2 &point) {
    LineSegment goal = getGoalSides(field, ourGoal);
    Angle angleLeft = Angle(goal.start - point);
    Angle angleRight = Angle(goal.end - point);
    return angleLeft.shortestAngleDiff(angleRight);
}

double FieldComputations::getPercentageOfGoalVisibleFromPoint(const rtt_world::Field &field, bool ourGoal, const Vector2 &point, rtt::world::view::WorldDataView world, int id,
                                                              bool ourTeam) {
    double goalWidth = field.getGoalWidth();
    double blockadeLength = 0;
    auto &robots = ourTeam ? world.getThem() : world.getUs();
    for (auto const &blockade : getBlockadesMappedToGoal(field, ourGoal, point, robots, id, ourTeam)) {
        blockadeLength += blockade.start.dist(blockade.end);
    }
    return fmax(100 - blockadeLength / goalWidth * 100, 0.0);
}

std::vector<LineSegment> FieldComputations::getVisiblePartsOfGoal(const rtt_world::Field &field, bool ourGoal, const Vector2 &point,
                                                                  const std::vector<rtt::world::view::RobotView> &robots) {
    // TODO: improve when it takes our/their robots into account
    std::vector<LineSegment> blockades = getBlockadesMappedToGoal(field, ourGoal, point, robots, -1, true);
    LineSegment goalSide = getGoalSides(field, ourGoal);
    double goalX = goalSide.start.x;  // The x-coordinate of the entire goal line (all vectors on this line have the same x-coordinate).
    double upperGoalY = goalSide.end.y;
    double lowerY = goalSide.start.y;
    std::vector<LineSegment> visibleParts = {};

    // The obstacles are sorted on their smallest y value. We start from the lowest goal side at the start as lowerY value and everytime we add a vector from the lowest goalside to
    // the lowest part of the obstacle and we remember the upper part of the obstacle. That upper part is stored as the lowerY value again and we can repeat the same process.
    for (auto const &blockade : blockades) {
        auto lowerbound = blockade.start.y;

        // If the lowerbound is the same as the lowerY value then the visible part has a length of 0 and we don't care about it. Originally used to be != but floating point errors
        // are tears, i.e. rounding of floating points might turn two same float values to different values.
        if (fabs(lowerbound - lowerY) > NEGLIGIBLE_LENGTH) {
            visibleParts.emplace_back(LineSegment(Vector2(goalX, lowerY), Vector2(goalX, lowerbound)));
        }
        lowerY = blockade.end.y;
    }

    // If the last lowerY value is the same as the upper goal side then the last visible part has a length of 0 and we don't care about it.
    if (fabs(lowerY - upperGoalY) > NEGLIGIBLE_LENGTH) {
        visibleParts.emplace_back(LineSegment(Vector2(goalX, lowerY), Vector2(goalX, upperGoalY)));
    }
    return visibleParts;
}

LineSegment FieldComputations::getGoalSides(const rtt_world::Field &field, bool ourGoal) {
    if (ourGoal) {
        return LineSegment(field.getOurBottomGoalSide(), field.getOurTopGoalSide());
    } else {
        return LineSegment(field.getTheirBottomGoalSide(), field.getTheirTopGoalSide());
    }
}

double FieldComputations::getDistanceToGoal(const rtt_world::Field &field, bool ourGoal, const Vector2 &point) { return getGoalSides(field, ourGoal).distanceToLine(point); }

std::shared_ptr<Vector2> FieldComputations::lineIntersectionWithDefenseArea(const rtt_world::Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd,
                                                                            double margin, bool ignoreGoalLine) {
    auto defenseArea = getDefenseArea(field, ourGoal, margin, field.getBoundaryWidth());

    std::vector<Vector2> intersections;
    if (!ignoreGoalLine) {
        intersections = defenseArea.intersections({lineStart, lineEnd});
    } else {
        auto defenseAreaVertices = defenseArea.vertices;
        // Loop over the all lines of the defense area except the goal line and check for intersections
        for (size_t i = 0; i < defenseAreaVertices.size() - 1; i++) {
            auto intersection = LineSegment(defenseAreaVertices[i], defenseAreaVertices[i + 1]).intersects({lineStart, lineEnd});
            if (intersection) intersections.push_back(intersection.value());
        }
    }

    if (intersections.size() == 1) {
        return std::make_shared<Vector2>(intersections.at(0));
    } else if (intersections.size() == 2) {
        double distanceFirstIntersection = lineStart.dist(intersections.at(0));
        double distanceSecondIntersection = lineStart.dist(intersections.at(1));
        return std::make_shared<Vector2>(distanceFirstIntersection < distanceSecondIntersection ? intersections.at(0) : intersections.at(1));
    } else {
        return nullptr;
    }
}

std::optional<Vector2> FieldComputations::lineIntersectionWithField(const rtt_world::Field &field, const Vector2 &lineStart, const Vector2 &lineEnd, double margin) {
    auto fieldEdges = getFieldEdge(field, margin);
    auto intersections = fieldEdges.intersections({lineStart, lineEnd});

    if (intersections.size() == 1) {
        return intersections.at(0);
    } else if (intersections.size() == 2) {
        double distanceFirstIntersection = lineStart.dist(intersections.at(0));
        double distanceSecondIntersection = lineStart.dist(intersections.at(1));
        return distanceFirstIntersection < distanceSecondIntersection ? intersections.at(0) : intersections.at(1);
    } else {
        return std::nullopt;
    }
}

// True standard which mean field.getBoundaryWidth() is used otherwise margin is used
Polygon FieldComputations::getDefenseArea(const rtt_world::Field &field, bool ourDefenseArea, double margin, double backMargin) {
    Vector2 belowGoal = ourDefenseArea ? field.getBottomLeftOurDefenceArea() : field.getBottomRightTheirDefenceArea();
    Vector2 aboveGoal = ourDefenseArea ? field.getTopLeftOurDefenceArea() : field.getTopRightTheirDefenceArea();
    Vector2 bottomPenalty = ourDefenseArea ? field.getLeftPenaltyLineBottom() : field.getRightPenaltyLineBottom();
    Vector2 topPenalty = ourDefenseArea ? field.getLeftPenaltyLineTop() : field.getRightPenaltyLineTop();

    if (aboveGoal.y < belowGoal.y) {
        std::swap(aboveGoal, belowGoal);
    }
    if (topPenalty.y < bottomPenalty.y) {
        std::swap(topPenalty, bottomPenalty);
    }

    if (ourDefenseArea) {
        if (aboveGoal.x > topPenalty.x) {
            std::swap(aboveGoal, topPenalty);
        }
        if (belowGoal.x > bottomPenalty.x) {
            std::swap(belowGoal, bottomPenalty);
        }
    } else {
        if (aboveGoal.x < topPenalty.x) {
            std::swap(aboveGoal, topPenalty);
        }
        if (belowGoal.x < bottomPenalty.x) {
            std::swap(belowGoal, bottomPenalty);
        }
    }

    belowGoal = ourDefenseArea ? belowGoal + Vector2(-backMargin, -margin) : belowGoal + Vector2(backMargin, -margin);
    aboveGoal = ourDefenseArea ? aboveGoal + Vector2(-backMargin, margin) : aboveGoal + Vector2(backMargin, margin);
    bottomPenalty = ourDefenseArea ? bottomPenalty + Vector2(margin, -margin) : bottomPenalty + Vector2(-margin, -margin);
    topPenalty = ourDefenseArea ? topPenalty + Vector2(margin, margin) : topPenalty + Vector2(-margin, margin);

    std::vector<Vector2> defenseArea = {belowGoal, bottomPenalty, topPenalty, aboveGoal};
    //interface::Input::drawDebugData(defenseArea);
    return {defenseArea};
}

Polygon FieldComputations::getGoalArea(const rtt_world::Field &field, bool ourGoal, double margin, bool hasBackMargin) {
    double goalDepth = hasBackMargin ? field.getGoalDepth() + margin : field.getGoalDepth();
    Vector2 outerBottomGoal = ourGoal ? field.getOurBottomGoalSide() + Vector2(margin, -margin) : field.getTheirBottomGoalSide() + Vector2(-margin, -margin);
    Vector2 innerBottomGoal = ourGoal ? field.getOurBottomGoalSide() + Vector2(-goalDepth, -margin) : field.getTheirBottomGoalSide() + Vector2(goalDepth, -margin);
    Vector2 innerTopGoal = ourGoal ? field.getOurTopGoalSide() + Vector2(-goalDepth, margin) : field.getTheirTopGoalSide() + Vector2(goalDepth, margin);
    Vector2 outerTopGoal = ourGoal ? field.getOurTopGoalSide() + Vector2(margin, margin) : field.getTheirTopGoalSide() + Vector2(-margin, margin);

    std::vector<Vector2> goalArea = {outerBottomGoal, innerBottomGoal, innerTopGoal, outerTopGoal};
    //interface::Input::drawDebugData(goalArea, ourGoal ? Qt::green : Qt::red, interface::Drawing::LINES_CONNECTED);
    return Polygon(goalArea);
}

Polygon FieldComputations::getFieldEdge(const rtt_world::Field &field, double margin) {
    std::vector<Vector2> fieldEdge = {field.getBottomLeftCorner() + Vector2(-margin, -margin), field.getTopLeftCorner() + Vector2(-margin, margin),
                                      field.getTopRightCorner() + Vector2(margin, margin), field.getBottomRightCorner() + Vector2(margin, -margin)};
    //interface::Input::drawDebugData(fieldEdge, Qt::red, interface::Drawing::LINES_CONNECTED);
    return Polygon(fieldEdge);
}

std::vector<LineSegment> FieldComputations::getBlockadesMappedToGoal(const rtt_world::Field &field, bool ourGoal, const Vector2 &point,
                                                                     const std::vector<rtt::world::view::RobotView> &robots, int id, bool ourTeam) {
    std::vector<LineSegment> blockades = {};
    const double robotRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    LineSegment goalSide = getGoalSides(field, ourGoal);
    for (auto const &robot : robots) {
        std::optional<LineSegment> blockade = robotBlockade(ourGoal, point, id, ourTeam, robot, robotRadius, goalSide);
        if (blockade.has_value()) {
            blockades.emplace_back(blockade.value());
        }
    }
    return mergeBlockades(blockades);
}

std::optional<LineSegment> FieldComputations::robotBlockade(bool ourGoal, const Vector2 &point, int id, bool ourTeam, const rtt::world::view::RobotView robot,
                                                            const double robotRadius, LineSegment goalSide) {
    // Discard the robot if it belong to the same team or if it has the given id.
    if (robot->getId() == id || robot->getTeam() == (ourTeam ? rtt::world::Team::us : rtt::world::Team::them)) return {};

    // Discard already the robot if it is not between the goal and point, or if the robot is standing on this point.
    double lenToBot = (point - robot->getPos()).length();
    bool isRobotItself = lenToBot <= robotRadius;
    bool isInPotentialBlockingZone = ourGoal ? robot->getPos().x < point.x + robotRadius : robot->getPos().x > point.x - robotRadius;
    if (isRobotItself || !isInPotentialBlockingZone) return {};

    // Compute the shadow caused by the robot on the goal side
    double theta = asin(robotRadius / lenToBot);
    double length = sqrt(lenToBot * lenToBot - robotRadius * robotRadius);
    Vector2 lowerSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->getPos()) - point).angle() - theta);
    Vector2 upperSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->getPos()) - point).angle() + theta);
    return Shadow::shadow(point, LineSegment(lowerSideOfRobot, upperSideOfRobot), goalSide, NEGLIGIBLE_LENGTH);
}

std::vector<LineSegment> FieldComputations::mergeBlockades(std::vector<LineSegment> blockades) {
    /* If two blockades intersect (in this case, overlap), we take the beginning of the first obstacle and the end of the second obstacle, and put them back in the front of the
     * obstacles vector. The second element gets erased. If they don't intersect, try the next two obstacles. Repeat this procedure until no overlaps are left. */
    std::sort(blockades.begin(), blockades.end(), [](const LineSegment &a, const LineSegment &b) { return a.start.y < b.start.y; });
    int iterator = 0;
    while (iterator < static_cast<int>(blockades.size()) - 1) {
        LineSegment &firstBlockade = blockades.at(iterator);
        LineSegment &secondBlockade = blockades.at(iterator + 1);
        if (firstBlockade.end.y >= secondBlockade.start.y) {
            // If the first two elements intersects, then merge these blockades into 1 single blockade.
            auto upperbound = fmax(firstBlockade.end.y, secondBlockade.end.y);
            auto newBlockade = LineSegment(firstBlockade.start, Vector2(firstBlockade.start.x, upperbound));
            blockades.erase(blockades.begin() + iterator + 1);
            blockades.at(iterator) = newBlockade;
        } else {
            iterator++;
        }
    }
    return blockades;
}

Vector2 FieldComputations::projectPointInField(const world::Field &field, Vector2 point, double margin) {
    Vector2 projectedPoint;
    projectedPoint.x = std::clamp(point.x, field.getLeftmostX() + margin + PROJECTION_MARGIN, field.getRightmostX() - margin - PROJECTION_MARGIN);
    projectedPoint.y = std::clamp(point.y, field.getBottommostY() + margin + PROJECTION_MARGIN, field.getTopmostY() - margin - PROJECTION_MARGIN);
    return projectedPoint;
}

Vector2 FieldComputations::projectPointOutOfDefenseArea(const world::Field &field, Vector2 point, double ourDefenseAreaMargin, double theirDefenseAreaMargin) {
    if (pointIsInField(field, point) && !pointIsInTheirDefenseArea(field, point, theirDefenseAreaMargin) && !pointIsInOurDefenseArea(field, point, ourDefenseAreaMargin))
        return point;

    // If the point is not in the field yet, project it into the field
    if (!pointIsInField(field, point)) {
        point = projectPointInField(field, point);
    }

    // Calculate how far the point is from the defense area border in the x and y direction
    double xDiff;
    double yDiff;
    if (pointIsInOurDefenseArea(field, point, ourDefenseAreaMargin)) {
        xDiff = (field.getLeftPenaltyLineBottom().x + ourDefenseAreaMargin + PROJECTION_MARGIN) - point.x;
        yDiff = point.y > 0 ? (field.getLeftPenaltyLineTop().y + ourDefenseAreaMargin + PROJECTION_MARGIN) - point.y
                            : (field.getLeftPenaltyLineBottom().y - ourDefenseAreaMargin - PROJECTION_MARGIN) - point.y;
    } else if (pointIsInTheirDefenseArea(field, point, theirDefenseAreaMargin)) {
        xDiff = (field.getRightPenaltyLineBottom().x - theirDefenseAreaMargin - PROJECTION_MARGIN) - point.x;
        yDiff = point.y > 0 ? (field.getRightPenaltyLineTop().y + theirDefenseAreaMargin + PROJECTION_MARGIN) - point.y
                            : (field.getRightPenaltyLineBottom().y - theirDefenseAreaMargin - PROJECTION_MARGIN) - point.y;
    } else
        return point;  // In case it is in neither defense area, just return the point

    if (fabs(xDiff) < fabs(yDiff)) return {point.x + xDiff, point.y};
    return {point.x, point.y + yDiff};
}

Vector2 FieldComputations::projectPointToValidPosition(const world::Field &field, Vector2 point, stp::AvoidObjects avoidObjects, double fieldMargin, double ourDefenseAreaMargin,
                                                       double theirDefenseAreaMargin) {
    Vector2 projectedPos = point;
    if (avoidObjects.shouldAvoidOutOfField) projectedPos = projectPointInField(field, projectedPos);
    if (avoidObjects.shouldAvoidDefenseArea) projectedPos = projectPointOutOfDefenseArea(field, projectedPos, ourDefenseAreaMargin, theirDefenseAreaMargin);
    return projectedPos;
}

Vector2 FieldComputations::projectPointIntoFieldOnLine(const world::Field &field, Vector2 point, Vector2 p1, Vector2 p2, double fieldMargin) {
    auto projectedPos = LineSegment(p1, p2).project(point);
    if (FieldComputations::pointIsInField(field, projectedPos, fieldMargin)) return projectedPos;

    auto intersection_lhs = FieldComputations::lineIntersectionWithField(field, projectedPos, p1, fieldMargin - PROJECTION_MARGIN);
    auto intersection_rhs = FieldComputations::lineIntersectionWithField(field, projectedPos, p2, fieldMargin - PROJECTION_MARGIN);

    // If there is no point  on this line inside the field, project the position into the field and return it
    if (!intersection_lhs && !intersection_rhs) return projectPointInField(field, projectedPos, fieldMargin);
    double dist_lhs = intersection_lhs ? point.dist(*intersection_lhs) : std::numeric_limits<double>::max();
    double dist_rhs = intersection_rhs ? point.dist(*intersection_rhs) : std::numeric_limits<double>::max();
    return (dist_lhs < dist_rhs ? *intersection_lhs : *intersection_rhs);  // return the intersection closest to the point
}

Vector2 FieldComputations::projectPointToValidPositionOnLine(const world::Field &field, Vector2 point, Vector2 p1, Vector2 p2, stp::AvoidObjects avoidObjects, double fieldMargin,
                                                             double ourDefenseAreaMargin, double theirDefenseAreaMargin) {
    // Subtract PROJECTION_MARGIN to avoid the situation where the point is between the left field line and our goal line (-6.0 < x < -5.9)
    auto pointProjectedInField = projectPointIntoFieldOnLine(field, point, p1, p2, fieldMargin - PROJECTION_MARGIN);

    bool ourGoal;   // Which goal's defense area the projected point is in
    double margin;  // The margin to be used for the defense area- set to ourDefenseMargin or theirDefenseAreaMargin depending on where the projected pos is
    if (pointIsInOurDefenseArea(field, pointProjectedInField, ourDefenseAreaMargin)) {
        margin = ourDefenseAreaMargin;
        ourGoal = true;
    } else if (pointIsInTheirDefenseArea(field, pointProjectedInField, theirDefenseAreaMargin)) {
        margin = theirDefenseAreaMargin;
        ourGoal = false;
    } else {
        return pointProjectedInField;  // If the projected position is not in a defense area, return it
    }

    std::vector<Vector2> intersections;
    auto defenseAreaVertices = getDefenseArea(field, ourGoal, margin + PROJECTION_MARGIN, 0).vertices;
    // Loop over the all lines of the defense area except the goal line and check for intersections
    for (size_t i = 0; i < defenseAreaVertices.size() - 1; i++) {
        auto intersection = LineSegment(defenseAreaVertices[i], defenseAreaVertices[i + 1]).intersects({p1, p2});
        if (intersection) intersections.push_back(intersection.value());
    }

    // If there is no point  on this line outside the defense area, project the position out of the defense area and return it
    if (intersections.empty()) return projectPointOutOfDefenseArea(field, pointProjectedInField, margin);

    // Return the intersection closest to the given point
    return *std::min_element(intersections.begin(), intersections.end(), [&point](auto lhs, auto rhs) { return point.dist(lhs) < point.dist(rhs); });
}
};  // namespace rtt::ai
