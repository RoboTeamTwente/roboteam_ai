#include <roboteam_utils/Shadow.h>
#include "world/FieldComputations.h"

namespace rtt {
namespace ai {

using util = control::ControlUtils;

bool FieldComputations::pointIsInDefenseArea(const rtt_world::Field &field, const Vector2 &point, bool isOurDefenceArea, double margin, double backMargin) {
    auto defenseArea = FieldComputations::getDefenseArea(field, isOurDefenceArea, margin, backMargin);
    return defenseArea.contains(point);
}

bool FieldComputations::pointIsInDefenseArea(const rtt_world::Field &field, const Vector2 &point, bool isOurDefenceArea, double margin) {
    return pointIsInDefenseArea(field, point, isOurDefenceArea, margin, margin);
}

bool FieldComputations::pointIsInField(const rtt_world::Field &field, const Vector2 &point, double margin) {
    return (point.x <= field.getRightmostX() + margin && point.x >= field.getLeftmostX() - margin && point.y <= field.getTopmostY() + margin &&
            point.y >= field.getBottommostY() - margin);
}

bool FieldComputations::pointIsValidPosition(const rtt_world::Field &field, const Vector2 &point, double margin){
    return (!pointIsInDefenseArea(field, point, true, margin) && !pointIsInDefenseArea(field, point, false, margin) && pointIsInField(field, point, margin));
}

double FieldComputations::getTotalGoalAngle(const rtt_world::Field &field, bool ourGoal, const Vector2 &point) {
    LineSegment goal = getGoalSides(field, ourGoal);
    Angle angleLeft = Angle(goal.start - point);
    Angle angleRight = Angle(goal.end - point);
    return angleLeft.shortestAngleDiff(angleRight);
}

double FieldComputations::getPercentageOfGoalVisibleFromPoint(const rtt_world::Field &field, bool ourGoal, const Vector2 &point, rtt::world::view::WorldDataView &world, int id,
                                                                bool ourTeam) {
    double goalWidth = field.getGoalWidth();
    double blockadeLength = 0;
    for (auto const &blockade : getBlockadesMappedToGoal(field, ourGoal, point, world.getRobotsNonOwning(), id, ourTeam)) {
        blockadeLength += blockade.start.dist(blockade.end);
    }
    return fmax(100 - blockadeLength / goalWidth * 100, 0.0);
}

std::vector<LineSegment> FieldComputations::getVisiblePartsOfGoal(const rtt_world::Field &field, bool ourGoal, const Vector2 &point, rtt::world::view::WorldDataView &world) {
    return getVisiblePartsOfGoal(field, ourGoal, point, world.getUs());
}

std::vector<LineSegment> FieldComputations::getVisiblePartsOfGoal(const rtt_world::Field &field, bool ourGoal, const Vector2 &point,
                                                            const std::vector<rtt::world::view::RobotView> &robots) {
    std::vector<LineSegment> blockades = getBlockadesMappedToGoal(field, ourGoal, point, robots);
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

double FieldComputations::getDistanceToGoal(const rtt_world::Field &field, bool ourGoal, const Vector2 &point) {
    return getGoalSides(field, ourGoal).distanceToLine(point);
}

Vector2 FieldComputations::getPenaltyPoint(const rtt_world::Field &field, bool ourGoal) {
    if (ourGoal) {
        return field.getLeftPenaltyPoint();
    } else {
        return field.getRightPenaltyPoint();
    }
}

std::shared_ptr<Vector2> FieldComputations::lineIntersectionWithDefenceArea(const rtt_world::Field &field, bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd,
                                                                            double margin) {
    auto defenseArea = getDefenseArea(field, ourGoal, margin, field.getBoundaryWidth());
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

// True standard which mean field.getBoundaryWidth() is used otherwise margin is used
Polygon FieldComputations::getDefenseArea(const rtt_world::Field &field, bool ourDefenseArea, double margin, double backMargin) {
    Vector2 belowGoal = ourDefenseArea ? field.getBottomLeftOurDefenceArea() + Vector2(-backMargin, -margin)
                                            : field.getBottomRightTheirDefenceArea() + Vector2(backMargin, -margin);
    Vector2 aboveGoal = ourDefenseArea ? field.getTopLeftOurDefenceArea() + Vector2(-backMargin, margin) : field.getTopRightTheirDefenceArea() + Vector2(backMargin, margin);
    Vector2 bottomPenalty = ourDefenseArea ? field.getLeftPenaltyLineBottom() + Vector2(margin, -margin) : field.getRightPenaltyLineBottom() + Vector2(-margin, -margin);
    Vector2 topPenalty = ourDefenseArea ? field.getLeftPenaltyLineTop() + Vector2(margin, margin) : field.getRightPenaltyLineTop() + Vector2(-margin, margin);

    std::vector<Vector2> defenseArea = {bottomPenalty, topPenalty, aboveGoal, belowGoal};
    interface::Input::drawDebugData(defenseArea);
    return Polygon(defenseArea);
}

Polygon FieldComputations::getGoalArea(const rtt_world::Field &field, bool ourGoal, double margin, bool hasBackMargin) {
    double goalDepth = hasBackMargin ? field.getGoalDepth() + margin : field.getGoalDepth();
    Vector2 outerBottomGoal = ourGoal ? field.getOurBottomGoalSide() + Vector2(margin, -margin) : field.getTheirBottomGoalSide() + Vector2(-margin, -margin);
    Vector2 innerBottomGoal = ourGoal ? field.getOurBottomGoalSide() + Vector2(-goalDepth, -margin) : field.getTheirBottomGoalSide() + Vector2(goalDepth, -margin);
    Vector2 innerTopGoal = ourGoal ? field.getOurTopGoalSide() + Vector2(-goalDepth, margin) : field.getTheirTopGoalSide() + Vector2(goalDepth, margin);
    Vector2 outerTopGoal = ourGoal ? field.getOurTopGoalSide() + Vector2(margin, margin) : field.getTheirTopGoalSide() + Vector2(-margin, margin);

    std::vector<Vector2> goalArea = {outerBottomGoal, innerBottomGoal, innerTopGoal, outerTopGoal};
    interface::Input::drawDebugData(goalArea, ourGoal ? Qt::green : Qt::red, interface::Drawing::LINES_CONNECTED);
    return Polygon(goalArea);
}

Polygon FieldComputations::getFieldEdge(const rtt_world::Field &field, double margin) {
    std::vector<Vector2> fieldEdge = {field.getBottomLeftCorner() + Vector2(-margin, -margin), field.getTopLeftCorner() + Vector2(-margin, margin),
                                      field.getTopRightCorner() + Vector2(margin, margin), field.getBottomRightCorner() + Vector2(margin, -margin)};
    interface::Input::drawDebugData(fieldEdge, Qt::red, interface::Drawing::LINES_CONNECTED);
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
    if (robot->getId() == id && robot->getTeam() == (ourTeam ? rtt::world::Team::us : rtt::world::Team::them)) return {};

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

Vector2 FieldComputations::placePointInField(const rtt_world::Field &field, const Vector2 &point){
    if (pointIsValidPosition(field,point)) return point;
    Vector2 fixedPoint = point;
    if (point.y > field.getTopLeftCorner().y) fixedPoint.y = field.getTopLeftCorner().y; //Top
    if (point.x > field.getBottomRightCorner().x) fixedPoint.x =field.getBottomRightCorner().x; //Right
    if (point.y > field.getBottomRightCorner().y) fixedPoint.y = field.getBottomRightCorner().y; //Bot
    if (point.x < field.getTopLeftCorner().x) fixedPoint.x = field.getTopLeftCorner().x; //Left
    return fixedPoint;
}

}  // namespace ai
}  // namespace rtt
