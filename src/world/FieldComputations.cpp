//
// Created by mrlukasbos on 19-10-18.
//

#include "world/FieldComputations.h"

namespace rtt {
namespace ai {

using util = control::ControlUtils;

bool FieldComputations::pointIsInDefenceArea(const Field &field, const Vector2 &point, bool isOurDefenceArea, double margin, bool includeOutsideField) {
    auto defenseArea = FieldComputations::getDefenseArea(field, isOurDefenceArea, margin, includeOutsideField);
    return defenseArea.contains(point);
}

bool FieldComputations::pointIsInField(const Field &field, const Vector2 &point, double margin) {
  return (point.x <= field.getRightmostX() + margin && point.x >= field.getLeftmostX() - margin
            && point.y <= field.getTopmostY() + margin && point.y >= field.getBottommostY() - margin);
}

double FieldComputations::getTotalGoalAngle(const Field &field, bool ourGoal, const Vector2 &point) {
    Line goal = getGoalSides(field, ourGoal);
    double angleLeft = (goal.start - point).angle();
    double angleRight = (goal.end - point).angle();
    return control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleLeft), control::ControlUtils::constrainAngle(angleRight));
}

double FieldComputations::getPercentageOfGoalVisibleFromPoint(const Field &field, bool ourGoal, const Vector2 &point, const world::WorldData &data, int id, bool ourTeam) {
    double goalWidth = field.getGoalWidth();
    double blockadeLength = 0;
    for (auto const &blockade : getBlockadesMappedToGoal(field, ourGoal, point, data, id, ourTeam)) {
        blockadeLength += blockade.start.dist(blockade.end);
    }
    return fmax(100 - blockadeLength / goalWidth * 100, 0.0);
}

std::vector<Line> FieldComputations::getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2 &point, const world::WorldData &data) {
    std::vector<LineSegment> blockades = getBlockadesMappedToGoal(field, ourGoal, point, data);
    Line goalSide = getGoalSides(field, ourGoal);
    double goalX = goalSide.start.x; // The x-coordinate of the entire goal line (all vectors on this line have the same x-coordinate).
    double upperGoalY = goalSide.end.y;
    double lowerY = goalSide.start.y;
    std::vector<Line> visibleParts = {};

    // The obstacles are sorted on their smallest y value. We start from the lowest goal side at the start as lowerY value and everytime we add a vector from the lowest goalside to
    // the lowest part of the obstacle and we remember the upper part of the obstacle. That upper part is stored as the lowerY value again and we can repeat the same process.
    for (auto const &blockade : blockades) {
        auto lowerbound = blockade.start.y;

        // If the lowerbound is the same as the lowerY value then the visible part has a length of 0 and we don't care about it. Originally used to be != but floating point errors
        // are tears, i.e. rounding of floating points might turn two same float values to different values.
        if (fabs(lowerbound - lowerY) > NEGLIGIBLE_LENGTH) {
            visibleParts.emplace_back(Line(Vector2(goalX, lowerY), Vector2(goalX, lowerbound)));
        }
        lowerY = blockade.end.y;
    }

    // If the last lowerY value is the same as the upper goal side then the last visible part has a length of 0 and we don't care about it.
    if (fabs(lowerY - upperGoalY) > NEGLIGIBLE_LENGTH) {
        visibleParts.emplace_back(Line(Vector2(goalX, lowerY), Vector2(goalX, upperGoalY)));
    }
    return visibleParts;
}

Line FieldComputations::getGoalSides(const Field &field, bool ourGoal) {
    if (ourGoal) {
        return Line(field.getOurBottomGoalSide(), field.getOurTopGoalSide());
    } else {
        return Line(field.getTheirBottomGoalSide(), field.getTheirTopGoalSide());
    }
}

double FieldComputations::getDistanceToGoal(const Field &field, bool ourGoal, const Vector2 &point) {
    auto sides = getGoalSides(field, ourGoal);
    return control::ControlUtils::distanceToLineWithEnds(point, sides.start, sides.end);
}

Vector2 FieldComputations::getPenaltyPoint(const Field &field, bool ourGoal) {
    if (ourGoal) {
        return field.getLeftPenaltyPoint();
    } else {
        return field.getRightPenaltyPoint();
    }
}

std::shared_ptr<Vector2> FieldComputations::lineIntersectionWithDefenceArea(const Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd, double margin) {
    auto defenseArea = getDefenseArea(field, ourGoal, margin);
    auto intersections = defenseArea.intersections({lineStart, lineEnd});

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

Polygon FieldComputations::getDefenseArea(const Field &field, bool ourDefenseArea, double margin, bool includeOutSideField) {
    double backLineChanges = includeOutSideField ? field.getBoundaryWidth() : margin;
    Vector2 bottomGoal = ourDefenseArea ? field.getOurBottomGoalSide() + Vector2(-backLineChanges, -margin) : field.getTheirBottomGoalSide() + Vector2(backLineChanges, -margin);
    Vector2 topGoal = ourDefenseArea ? field.getOurTopGoalSide() + Vector2(-backLineChanges, margin) : field.getTheirTopGoalSide() + Vector2(backLineChanges, margin);
    Vector2 bottomPenalty = ourDefenseArea ? field.getLeftPenaltyLineBottom() + Vector2(margin, -margin) : field.getRightPenaltyLineBottom() + Vector2(-margin, -margin);
    Vector2 topPenalty = ourDefenseArea ? field.getLeftPenaltyLineTop() + Vector2(margin, margin) : field.getRightPenaltyLineTop() + Vector2(-margin, margin);

    std::vector<Vector2> defenseArea = {bottomPenalty, topPenalty, topGoal, bottomGoal};
    interface::Input::drawDebugData(defenseArea);
    return Polygon(defenseArea);
}

Polygon FieldComputations::getGoalArea(const Field &field, bool ourGoal, double margin, bool hasBackMargin) {
    double goalDepth = hasBackMargin ? field.getGoalDepth() + margin : field.getGoalDepth();
    Vector2 outerBottomGoal = ourGoal ? field.getOurBottomGoalSide() + Vector2(margin, -margin) : field.getTheirBottomGoalSide() + Vector2(-margin, -margin);
    Vector2 innerBottomGoal = ourGoal ? field.getOurBottomGoalSide() + Vector2(-goalDepth, -margin) : field.getTheirBottomGoalSide() + Vector2(goalDepth, -margin);
    Vector2 innerTopGoal = ourGoal ? field.getOurTopGoalSide() + Vector2(-goalDepth, margin) : field.getTheirTopGoalSide() + Vector2(goalDepth, margin);
    Vector2 outerTopGoal = ourGoal ? field.getOurTopGoalSide() + Vector2(margin, margin) : field.getTheirTopGoalSide() + Vector2(-margin, margin);

    std::vector<Vector2> goalArea = {outerBottomGoal, innerBottomGoal, innerTopGoal, outerTopGoal};
    interface::Input::drawDebugData(goalArea, ourGoal ? Qt::green : Qt::red, interface::Drawing::LINES_CONNECTED);
    return Polygon(goalArea);
}

Polygon FieldComputations::getFieldEdge(const Field &field, double margin) {
    std::vector<Vector2> fieldEdge = {field.getBottomLeftCorner() + Vector2(-margin, -margin), field.getTopLeftCorner() + Vector2(-margin, margin),
                                      field.getTopRightCorner() + Vector2(margin, margin), field.getBottomRightCorner() + Vector2(margin, -margin)};
    interface::Input::drawDebugData(fieldEdge, Qt::red, interface::Drawing::LINES_CONNECTED);
    return Polygon(fieldEdge);
}

std::vector<LineSegment> FieldComputations::getBlockadesMappedToGoal(const Field &field, bool ourGoal, const Vector2 &point, const world::WorldData &data, int id, bool ourTeam) {
    std::vector<LineSegment> blockades = {};
    const double robotRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();
    auto goalSide = getGoalSides(field, ourGoal);
    auto robots = data.us;
    robots.insert(robots.begin(), data.them.begin(), data.them.end());
    for (auto const &robot : robots) {
        std::optional<LineSegment> blockade = robotBlockade(ourGoal, point, id, ourTeam, robot, robotRadius, LineSegment(goalSide));
        if (blockade.has_value()) {
            blockades.emplace_back(blockade.value());
        }
    }
    return mergeBlockades(blockades);
}

std::optional<LineSegment> FieldComputations::robotBlockade(bool ourGoal, const Vector2 &point, int id, bool ourTeam, std::shared_ptr<Robot> robot,
                                                            const double robotRadius, LineSegment goalSide) {
    // Discard the robot if it belong to the same team or if it has the given id.
    if (robot->id == id && robot->team == (ourTeam ? Team::us : Team::them)) return {};

    // Discard already the robot if it is not between the goal and point, or if the robot is standing on this point.
    double lenToBot = (point - robot->pos).length();
    bool isRobotItself = lenToBot <= robotRadius;
    bool isInPotentialBlockingZone = ourGoal ? robot->pos.x < point.x + robotRadius : robot->pos.x > point.x - robotRadius;
    if (isRobotItself || !isInPotentialBlockingZone) return {};

    /* Check which part of the goal is blocked by this robot, by creating vectors from the point to the places that intersect with the robot and expanding these vectors to an
     * infinite line that intersects with the infinite line expansion of the goal side. After which intersection point of these lines is found and a projection is applied between
     * the intersection point and the goal side. Note that this projection is used, because the intersection point might not be on the goal side. In which case the point has to be
     * mapped to the bottommost or topmost position of the goal side. You can compare this code with computing the 'shadow' on the goal caused by the robot if the point is
     * considered a 'light source'. Note also that we do not have to check whether the lines go parallel, because the isRobitItself and isInPotentialBlockingZone check guarantees
     * that the lines will eventually intersect with each other and by this same check we know that the lines intersect in the right direction, i.e. if the robot was not in between
     * that goal and the point then the lines can also intersect with the infinite line expansion of the goal side but never causes a blockade 'shadow' on the goal side. */
    double theta = asin(robotRadius / lenToBot);
    double length = sqrt(lenToBot * lenToBot - robotRadius * robotRadius);
    Vector2 lowerSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->pos) - point).angle() - theta);
    Vector2 upperSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->pos) - point).angle() + theta);
    Vector2 lowerMapToGoal = goalSide.project(util::twoLineIntersection(point, lowerSideOfRobot, goalSide.start, goalSide.end));
    Vector2 upperMapToGoal = goalSide.project(util::twoLineIntersection(point, upperSideOfRobot, goalSide.start, goalSide.end));

    LineSegment blockade = LineSegment(lowerMapToGoal, upperMapToGoal);
    if (blockade.length() > NEGLIGIBLE_LENGTH) {
        return blockade;
    } else {
        return {}; // Ignore blockades 'shadows' that are entirely below/above the goal side.
    }
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

}  // namespace ai
}  // namespace rtt
