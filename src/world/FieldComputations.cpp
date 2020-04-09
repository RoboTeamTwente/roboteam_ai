#include "world/FieldComputations.h"
#include <control/ControlUtils.h>
#include <interface/api/Input.h>
#include "world_new/views/RobotView.hpp"
#include "world_new/views/WorldDataView.hpp"

namespace rtt {
namespace ai {

using util = control::ControlUtils;

bool FieldComputations::pointIsInDefenseArea(const Field &field, const Vector2 &point, bool isOurDefenceArea, double margin, bool includeOutsideField) {
    auto defenseArea = FieldComputations::getDefenseArea(field, isOurDefenceArea, margin, includeOutsideField);
    return defenseArea.contains(point);
}

// the margin is pointed inside the field!
bool FieldComputations::pointIsInField(const Field &field, const Vector2 &point, double margin) {
    return (point.x <= field.getRightmostX() - margin && point.x >= field.getLeftmostX() + margin && point.y <= field.getTopmostY() - margin &&
            point.y >= field.getBottommostY() + margin);
}

/// returns the angle the goal points make from a point
double FieldComputations::getTotalGoalAngle(const Field &field, bool ourGoal, const Vector2 &point) {
    Line goal = getGoalSides(field, ourGoal);
    double angleLeft = (goal.start - point).angle();
    double angleRight = (goal.end - point).angle();
    return control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleLeft), control::ControlUtils::constrainAngle(angleRight));
}

/// id and ourteam are for a robot not to be taken into account.
double FieldComputations::getPercentageOfGoalVisibleFromPoint(const Field &field, bool ourGoal, const Vector2 &point, world_new::view::WorldDataView &world, int id, bool ourTeam) {
    double goalWidth = field.getGoalWidth();
    double blockadeLength = 0;
    for (auto const &blockade : getBlockadesMappedToGoal(field, ourGoal, point, world.getRobotsNonOwning(), id, ourTeam)) {
        blockadeLength += blockade.start.dist(blockade.end);
    }
    return fmax(100 - blockadeLength / goalWidth * 100, 0.0);
}

std::vector<Line> FieldComputations::getBlockadesMappedToGoal(const Field &field, bool ourGoal, const Vector2 &point, std::vector<world_new::view::RobotView> robots, int id, bool ourTeam) {
    const double robotRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();

    Vector2 lowerGoalSide, upperGoalSide;
    auto sides = getGoalSides(field, ourGoal);
    lowerGoalSide = sides.start;
    upperGoalSide = sides.end;

    std::vector<Line> blockades = {};

    // all the obstacles should be robots
    for (auto const &robot : robots) {
        if (robot->getId() == id && robot->getTeam() == (ourTeam ? world_new::Team::us : world_new::Team::them)) continue;
        auto pos = robot->getPos();
        double lenToBot = (point - pos).length();
        // discard already all robots that are not at all between the goal and point, or if a robot is standing on this point
        bool isRobotItself = lenToBot <= robotRadius;
        bool isInPotentialBlockingZone = ourGoal ? pos.x < point.x + robotRadius : pos.x > point.x - robotRadius;
        if (!isRobotItself && isInPotentialBlockingZone) {
            // get the left and right sides of the robot
            double theta = asin(robotRadius / lenToBot);
            double length = sqrt(lenToBot * lenToBot - robotRadius * robotRadius);
            Vector2 lowerSideOfRobot = point + Vector2(length, 0).rotate((Vector2(pos) - point).angle() - theta);
            Vector2 upperSideOfRobot = point + Vector2(length, 0).rotate((Vector2(pos) - point).angle() + theta);
            // map points onto goal line

            // the forwardIntersection returns a double which is the scale of the vector projection
            // this returns -1.0 if there is no intersections in the forward direction
            double point1val = util::twoLineForwardIntersection(point, lowerSideOfRobot, lowerGoalSide, upperGoalSide);
            double point2val = util::twoLineForwardIntersection(point, upperSideOfRobot, lowerGoalSide, upperGoalSide);
            // Here is how we calculate the actual intersections using the above values
            Vector2 point1 = point + (lowerSideOfRobot - point) * point1val;
            Vector2 point2 = point + (upperSideOfRobot - point) * point2val;
            // we can use the

            // remove all obstacles that are completely out of the goal regardless

            bool validObstacle;
            // object completely faced the wrong way
            if (point1val <= 0 && point2val <= 0) {
                validObstacle = false;
            }
            // these following 2 cases are identical in logic but mirrored; one point hits the backline, other does not.
            // in that case, we pick the appropriate goalPost which would be right for the obstacle as new Point and check if this interval is valid
            else if (point1val <= 0 && point2val > 0) {
                validObstacle = true;
                if (point1.y < point2.y) {
                    point1 = upperGoalSide;
                } else {
                    point1 = lowerGoalSide;
                }
            } else if (point2val <= 0 && point1val > 0) {
                validObstacle = true;
                if (point2.y < point1.y) {
                    point2 = upperGoalSide;
                } else {
                    point2 = lowerGoalSide;
                }
            } else {
                //'normal' obstacle; check if the points are at good points
                validObstacle = true;
            }

            // check if both points are below or above the goal (this invalidates it again)
            if (validObstacle) {
                bool bothPointsBelowGoal = point1.y <= lowerGoalSide.y && point2.y <= lowerGoalSide.y;
                bool bothPointAboveGoal = point1.y >= upperGoalSide.y && point2.y >= upperGoalSide.y;
                if (bothPointsBelowGoal || bothPointAboveGoal) {
                    validObstacle = false;
                }
            }
            if (validObstacle) {
                // constrain the blockades to within the goal
                if (point1.y > point2.y) {  // point1 is largest
                    point1.y = fmin(point1.y, upperGoalSide.y);
                    point2.y = fmax(point2.y, lowerGoalSide.y);
                    // the first element in the pair is the smallest
                    blockades.emplace_back(Line(point2, point1));
                } else {  // point2 is largest
                    point2.y = fmin(point2.y, upperGoalSide.y);
                    point1.y = fmax(point1.y, lowerGoalSide.y);
                    // the first element in the pair is the smallest
                    blockades.emplace_back(Line(point1, point2));
                }
            }
        }
    }
    return mergeBlockades(blockades);
}

/*
 * if two blockades intersect (in this case, overlap), we take the beginning of the first
 * obstacle and the end of the second obstacle, and put them back in the front of the obstacles vector.
 * The second element gets erased. if they don't intersect, try the next two obstacles.
 * repeat until no overlaps are left.
 */
std::vector<Line> FieldComputations::mergeBlockades(std::vector<Line> blockades) {
    // sort the blockades from low to high
    std::sort(blockades.begin(), blockades.end(), [](const Line &a, const Line &b) { return a.start.y < b.start.y; });

    std::vector<Line> mergedBlockades;
    unsigned long iterator = 0;
    while (blockades.size() > (iterator + 1)) {
        if (blockades.at(iterator).end.y >= blockades.at(iterator + 1).start.y) {
            // if the first two elements intercept, merge them
            auto upperbound = fmax(blockades.at(iterator).end.y, blockades.at(iterator + 1).start.y);

            // construct a new vector from the lowest to highest blockade value
            auto newBlockade = Line(blockades.at(iterator).start, Vector2(blockades.at(iterator).start.x, upperbound));
            blockades.erase(blockades.begin() + iterator + 1);
            blockades.at(iterator) = newBlockade;
        } else {
            //  if they don't intercept, move on to the next obstacle
            iterator++;
        }
    }
    return blockades;
}

// Returns the sides of the goal. The first vector is the the lower side and the second is the upper side.
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
    } else if (intersections.size() > 1) {
        double closestIntersectionToLineStart = INT_MAX;
        Vector2 closestIntersection = intersections.at(0);
        for (auto const &intersection : intersections) {
            if (lineStart.dist(intersection) < closestIntersectionToLineStart) {
                closestIntersection = intersection;
                closestIntersectionToLineStart = lineStart.dist(intersection);
            }
        }
        return std::make_shared<Vector2>(closestIntersection);
    }
    return nullptr;
}

Polygon FieldComputations::getDefenseArea(const Field &field, bool ourDefenseArea, double margin, bool includeOutSideField) {
    double backLineUsXCoordinate = includeOutSideField ? field.getLeftmostX() - field.getBoundaryWidth() : field.getLeftmostX() - margin;
    double backLineThemXCoordinate = includeOutSideField ? field.getRightmostX() + field.getBoundaryWidth() : field.getRightmostX() + margin;

    std::vector<Vector2> defenceAreaUsPoints = {{field.getLeftPenaltyLine().begin.x + margin, field.getLeftPenaltyLine().begin.y - margin},
                                                {field.getLeftPenaltyLine().end.x + margin, field.getLeftPenaltyLine().end.y + margin},
                                                {backLineUsXCoordinate, field.getLeftPenaltyLine().end.y + margin},
                                                {backLineUsXCoordinate, field.getLeftPenaltyLine().begin.y - margin}};

    interface::Input::drawDebugData(defenceAreaUsPoints);
    Polygon defenceAreaUs(defenceAreaUsPoints);

    std::vector<Vector2> defenceAreaThemPoints = {{field.getRightPenaltyLine().begin.x - margin, field.getRightPenaltyLine().begin.y - margin},
                                                  {field.getRightPenaltyLine().end.x - margin, field.getRightPenaltyLine().end.y + margin},
                                                  {backLineThemXCoordinate, field.getRightPenaltyLine().end.y + margin},
                                                  {backLineThemXCoordinate, field.getRightPenaltyLine().begin.y - margin}};

    Polygon defenceAreaThem(defenceAreaThemPoints);
    return ourDefenseArea ? defenceAreaUs : defenceAreaThem;
}

Polygon FieldComputations::getGoalArea(const Field &field, bool ourGoal, double margin, bool hasBackMargin) {
    double marginBackside = hasBackMargin ? margin : 0.0;
    auto goalDepth = field.getGoalDepth() + marginBackside;

    if (ourGoal) {
        auto ourGoalSides = getGoalSides(field, true);
        std::vector<Vector2> areaUsPoints = {{ourGoalSides.start.x + margin, ourGoalSides.start.y - margin},
                                             {ourGoalSides.start.x - goalDepth, ourGoalSides.start.y - margin},
                                             {ourGoalSides.end.x - goalDepth, ourGoalSides.end.y + margin},
                                             {ourGoalSides.end.x + margin, ourGoalSides.end.y + margin}};

        interface::Input::drawDebugData(areaUsPoints, Qt::green, interface::Drawing::LINES_CONNECTED);
        return Polygon(areaUsPoints);
    }

    auto theirGoalSides = getGoalSides(field, false);
    std::vector<Vector2> areaThemPoints = {{theirGoalSides.start.x - margin, theirGoalSides.start.y - margin},
                                           {theirGoalSides.start.x + goalDepth, theirGoalSides.start.y - margin},
                                           {theirGoalSides.end.x + goalDepth, theirGoalSides.end.y + margin},
                                           {theirGoalSides.end.x - margin, theirGoalSides.end.y + margin}};

    interface::Input::drawDebugData(areaThemPoints, Qt::red, interface::Drawing::LINES_CONNECTED);

    return Polygon(areaThemPoints);
}

Polygon FieldComputations::getFieldEdge(const Field &field, double margin) {
    double left = field.getLeftmostX() + margin;
    double right = field.getRightmostX() - margin;
    double bottom = field.getBottommostY() + margin;
    double top = field.getTopmostY() - margin;

    std::vector<Vector2> fieldEdge = {{left, bottom}, {left, top}, {right, top}, {right, bottom}};

    interface::Input::drawDebugData(fieldEdge, Qt::red, interface::Drawing::LINES_CONNECTED);

    return Polygon(fieldEdge);
}

std::vector<Line> FieldComputations::getVisiblePartsOfGoalByObstacles(const Field &field,
                                                           bool ourGoal,
                                                           const Vector2 &point,
                                                           const std::vector<world_new::view::RobotView>& robots) {
    auto blockades = getBlockadesMappedToGoal(field, ourGoal, point, robots);

    auto sides = getGoalSides(field, ourGoal);
    auto lower = sides.start;
    auto upper = sides.end;

    auto lowerHook = lower;
    std::vector<Line> visibleParts = {};

    // we start from the lowerhook, which is the lowest goal side at the start.
    // The obstacles are sorted on their smallest value.
    // everytime we add a vector from the lowest goalside to the lowest part of the obstacle we remember the upper part of the obstacle
    // That upper part is stored as the lowerhook again: and we can repeat the process
    for (auto const &blockade : blockades) {
        auto lowerbound = fmin(blockade.start.y, blockade.end.y);

        // if the lowerbound is the same as the lower hook then the visible part has a length of 0 and we don't care about it
        // originally used to be != but floating point errors are tears.
        if (fabs(lowerbound - lowerHook.y) > 0.000001) {
            visibleParts.emplace_back(Line(lowerHook, Vector2(blockade.start.x, lowerbound)));
        }
        auto upperbound = fmax(blockade.start.y, blockade.end.y);
        lowerHook = Vector2(blockade.start.x, upperbound);
    }

    // if the last lowerhook is the same as the upper goal side then the visible part has a length of 0 and we don't care about it
    if (lowerHook != upper) {
        visibleParts.emplace_back(Line(lowerHook, upper));
    }
    return visibleParts;
}


/*
 * Get the visible parts of a goal
 * This is the inverse of getting the blockades of a goal
 */
std::vector<Line> FieldComputations::getVisiblePartsOfGoal(const Field &field, bool ourGoal, const Vector2 &point, const world_new::view::WorldDataView &world) {
    return getVisiblePartsOfGoalByObstacles(field, ourGoal, point, world.getThem());
}


}  // namespace ai
}  // namespace rtt
