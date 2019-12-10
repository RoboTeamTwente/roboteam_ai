//
// Created by mrlukasbos on 19-10-18.
//

#include <interface/api/Input.h>
#include <control/ControlUtils.h>
#include "world/WorldData.h"
#include "world/FieldComputations.h"
#include "world/World.h"

namespace rtt {
namespace ai {
namespace world {

FieldComputations fieldObj;
FieldComputations *field = &fieldObj;

using util = control::ControlUtils;

bool FieldComputations::pointIsInDefenceArea(const Vector2 &point, bool isOurDefenceArea, double margin, bool includeOutsideField) {
  auto defenseArea = getDefenseArea(isOurDefenceArea, margin, includeOutsideField);
  return defenseArea.contains(point);
}

// the margin is pointed inside the field!
bool FieldComputations::pointIsInField(const Vector2 &point, double margin) {
  auto field = FieldMessage::get_field();
  return (point.x <= field[RIGHTMOST_X] - margin && point.x >= field[LEFTMOST_X] + margin
            && point.y <= field[TOPMOST_Y]- margin && point.y >= field[BOTTOMMOST_Y] + margin);
}

/// returns the angle the goal points make from a point
double FieldComputations::getTotalGoalAngle(bool ourGoal, const Vector2 &point) {
  std::pair<Vector2, Vector2> goal = getGoalSides(ourGoal);
  double angleLeft = (goal.first - point).angle();
  double angleRight = (goal.second - point).angle();
  return control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(angleLeft),
                                                control::ControlUtils::constrainAngle(angleRight));
}

/// id and ourteam are for a robot not to be taken into account.
double FieldComputations::getPercentageOfGoalVisibleFromPoint(bool ourGoal, const Vector2 &point, const WorldData &data, int id,
                                                  bool ourTeam) {
    auto _field = FieldMessage::get_field();
    double goalWidth = _field[GOAL_WIDTH];
    double blockadeLength = 0;
    for (auto const &blockade : getBlockadesMappedToGoal(ourGoal, point, data, id, ourTeam)) {
        blockadeLength += blockade.first.dist(blockade.second);
    }
    return fmax(100 - blockadeLength / goalWidth * 100, 0.0);
}

std::vector<std::pair<Vector2, Vector2>> FieldComputations::getBlockadesMappedToGoal(bool ourGoal, const Vector2 &point,
                                                                         const WorldData &data, int id, bool ourTeam) {
  const double robotRadius = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS();

  Vector2 lowerGoalSide, upperGoalSide;
  auto sides = getGoalSides(ourGoal);
  lowerGoalSide = sides.first;
  upperGoalSide = sides.second;

  std::vector<std::pair<Vector2, Vector2>> blockades = {};

  // get all the robots
  auto robots = data.us;
  robots.insert(robots.begin(), data.them.begin(), data.them.end());
  // all the obstacles should be robots
  for (auto const &robot : robots) {
    if (robot->id==id && robot->team==(ourTeam ? Team::us : Team::them)) continue;
    double lenToBot = (point - robot->pos).length();
    // discard already all robots that are not at all between the goal and point, or if a robot is standing on this point
    bool isRobotItself = lenToBot <= robotRadius;
    bool isInPotentialBlockingZone = ourGoal ? robot->pos.x < point.x + robotRadius : robot->pos.x
        > point.x - robotRadius;
    if (!isRobotItself && isInPotentialBlockingZone) {

      // get the left and right sides of the robot
      double theta = asin(robotRadius/lenToBot);
      double length = sqrt(lenToBot*lenToBot - robotRadius*robotRadius);
      Vector2 lowerSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->pos) - point).angle() - theta);
      Vector2 upperSideOfRobot = point + Vector2(length, 0).rotate((Vector2(robot->pos) - point).angle() + theta);
      // map points onto goal line

      // the forwardIntersection returns a double which is the scale of the vector projection
      // this returns -1.0 if there is no intersections in the forward direction
      double point1val = util::twoLineForwardIntersection(point, lowerSideOfRobot, lowerGoalSide, upperGoalSide);
      double point2val = util::twoLineForwardIntersection(point, upperSideOfRobot, lowerGoalSide, upperGoalSide);
      // Here is how we calculate the actual intersections using the above values
      Vector2 point1 = point + (lowerSideOfRobot - point)*point1val;
      Vector2 point2 = point + (upperSideOfRobot - point)*point2val;
      // we can use the

      // remove all obstacles that are completely out of the goal regardless

      bool validObstacle;
      //object completely faced the wrong way
      if (point1val <= 0 && point2val <= 0) {
        validObstacle = false;
      }
        //these following 2 cases are identical in logic but mirrored; one point hits the backline, other does not.
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
        if (point1.y > point2.y) { // point1 is largest
          point1.y = fmin(point1.y, upperGoalSide.y);
          point2.y = fmax(point2.y, lowerGoalSide.y);
          // the first element in the pair is the smallest
          blockades.emplace_back(std::make_pair(point2, point1));
        } else { // point2 is largest
          point2.y = fmin(point2.y, upperGoalSide.y);
          point1.y = fmax(point1.y, lowerGoalSide.y);
          // the first element in the pair is the smallest
          blockades.emplace_back(std::make_pair(point1, point2));
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
std::vector<std::pair<Vector2, Vector2>> FieldComputations::mergeBlockades(std::vector<std::pair<Vector2, Vector2>> blockades) {

  // sort the blockades from low to high
  std::sort(blockades.begin(), blockades.end(),
            [](const std::pair<Vector2, Vector2> &a, const std::pair<Vector2, Vector2> &b) {
              return a.first.y < b.first.y;
            });

  std::vector<std::pair<Vector2, Vector2>> mergedBlockades;
  unsigned long iterator = 0;
  while (blockades.size() > (iterator + 1)) {
    if (blockades.at(iterator).second.y >= blockades.at(iterator + 1).first.y) {

      // if the first two elements intercept, merge them
      auto upperbound = fmax(blockades.at(iterator).second.y, blockades.at(iterator + 1).second.y);

      // construct a new vector from the lowest to highest blockade value
      auto newBlockade = std::make_pair(blockades.at(iterator).first,
                                        Vector2(blockades.at(iterator).first.x, upperbound));
      blockades.erase(blockades.begin() + iterator + 1);
      blockades.at(iterator) = newBlockade;
    } else {
      //  if they don't intercept, move on to the next obstacle
      iterator++;
    }
  }
  return blockades;
}

/*
 * Get the visible parts of a goal
 * This is the inverse of getting the blockades of a goal
 */
std::vector<std::pair<Vector2, Vector2>> FieldComputations::getVisiblePartsOfGoal(bool ourGoal, const Vector2 &point,
                                                                      const WorldData &data) {
  auto blockades = getBlockadesMappedToGoal(ourGoal, point, data);

  auto sides = getGoalSides(ourGoal);
  auto lower = sides.first;
  auto upper = sides.second;

  auto lowerHook = lower;
  std::vector<std::pair<Vector2, Vector2>> visibleParts = {};


  // we start from the lowerhook, which is the lowest goal side at the start.
  // The obstacles are sorted on their smallest value.
  // everytime we add a vector from the lowest goalside to the lowest part of the obstacle we remember the upper part of the obstacle
  // That upper part is stored as the lowerhook again: and we can repeat the process
  for (auto const &blockade : blockades) {
    auto lowerbound = fmin(blockade.first.y, blockade.second.y);

    // if the lowerbound is the same as the lower hook then the visible part has a length of 0 and we don't care about it
    // originally used to be != but floating point errors are tears.
    if (fabs(lowerbound - lowerHook.y) > 0.000001) {
      visibleParts.emplace_back(std::make_pair(lowerHook, Vector2(blockade.first.x, lowerbound)));
    }
    auto upperbound = fmax(blockade.first.y, blockade.second.y);
    lowerHook = Vector2(blockade.first.x, upperbound);
  }

  // if the last lowerhook is the same as the upper goal side then the visible part has a length of 0 and we don't care about it
  if (lowerHook!=upper) {
    visibleParts.emplace_back(std::make_pair(lowerHook, upper));
  }
  return visibleParts;
}

// Returns the sides of the goal. The first vector is the the lower side and the second is the upper side.
std::pair<Vector2, Vector2> FieldComputations::getGoalSides(bool ourGoal) {
    FieldMessage _field = FieldMessage::get_field();
    if (ourGoal) {
        return std::make_pair(_field[OUR_BOTTOM_GOAL_SIDE], _field[OUR_TOP_GOAL_SIDE]);
    }
    else {
        return std::make_pair(_field[THEIR_BOTTOM_GOAL_SIDE], _field[THEIR_TOP_GOAL_SIDE]);
    }
}

double FieldComputations::getDistanceToGoal(bool ourGoal, const Vector2 &point) {
  auto sides = getGoalSides(ourGoal);
  return control::ControlUtils::distanceToLineWithEnds(point, sides.first, sides.second);
}

Vector2 FieldComputations::getPenaltyPoint(bool ourGoal) {
    if (ourGoal) {
        return FieldMessage::get_field()[LEFT_PENALTY_POINT];
    } else {
        return FieldMessage::get_field()[RIGHT_PENALTY_POINT];
    }
}

std::shared_ptr<Vector2> FieldComputations::lineIntersectionWithDefenceArea(bool ourGoal, const Vector2 &lineStart,
                                                                const Vector2 &lineEnd, double margin) {
  auto defenseArea = getDefenseArea(ourGoal, margin);
  auto intersections = defenseArea.intersections({lineStart, lineEnd});

  if (intersections.size()==1) {
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

bool FieldComputations::lineIntersectsWithDefenceArea(bool ourGoal, const Vector2 &lineStart, const Vector2 &lineEnd,
                                          double margin) {
  auto defenseArea = getDefenseArea(ourGoal, margin);
  return defenseArea.doesIntersect({lineStart, lineEnd});
}

Polygon FieldComputations::getDefenseArea(bool ourDefenseArea, double margin, bool includeOutSideField) {
  FieldMessage _field = FieldMessage::get_field();

  double backLineUsXCoordinate = includeOutSideField ? _field[LEFTMOST_X] - _field[BOUNDARY_WIDTH] :
                                    _field[LEFTMOST_X] - margin;
  double backLineThemXCoordinate = includeOutSideField ? _field[RIGHTMOST_X] + _field[BOUNDARY_WIDTH]:
                                    _field[RIGHTMOST_X] + margin;

  std::vector<Vector2> defenceAreaUsPoints = {
      {_field[LEFT_PENALTY_LINE].begin.x + margin, _field[LEFT_PENALTY_LINE].begin.y - margin},
      {_field[LEFT_PENALTY_LINE].end.x + margin, _field[LEFT_PENALTY_LINE].end.y + margin},
      {backLineUsXCoordinate, _field[LEFT_PENALTY_LINE].end.y + margin},
      {backLineUsXCoordinate, _field[LEFT_PENALTY_LINE].begin.y - margin}};

  interface::Input::drawDebugData(defenceAreaUsPoints);
  Polygon defenceAreaUs(defenceAreaUsPoints);

  std::vector<Vector2> defenceAreaThemPoints = {
      {_field[RIGHT_PENALTY_LINE].begin.x - margin, _field[RIGHT_PENALTY_LINE].begin.y - margin},
      {_field[RIGHT_PENALTY_LINE].end.x - margin, _field[RIGHT_PENALTY_LINE].end.y + margin},
      {backLineThemXCoordinate, _field[RIGHT_PENALTY_LINE].end.y + margin},
      {backLineThemXCoordinate, _field[RIGHT_PENALTY_LINE].begin.y - margin}};

  Polygon defenceAreaThem(defenceAreaThemPoints);
  return ourDefenseArea ? defenceAreaUs : defenceAreaThem;
}

Polygon FieldComputations::getGoalArea(bool ourGoal, double margin, bool hasBackMargin) {
  FieldMessage _field = FieldMessage::get_field();

  double marginBackside = hasBackMargin ? margin : 0.0;
  auto goalDepth = _field[GOAL_DEPTH] + marginBackside;

  if (ourGoal) {
    auto ourGoalSides = getGoalSides(true);
    std::vector<Vector2> areaUsPoints = {
        {ourGoalSides.first.x + margin, ourGoalSides.first.y - margin},
        {ourGoalSides.first.x - goalDepth, ourGoalSides.first.y - margin},
        {ourGoalSides.second.x - goalDepth, ourGoalSides.second.y + margin},
        {ourGoalSides.second.x + margin, ourGoalSides.second.y + margin}};

    interface::Input::drawDebugData(areaUsPoints, Qt::green, interface::Drawing::LINES_CONNECTED);
    return Polygon(areaUsPoints);
  }

  auto theirGoalSides = getGoalSides(false);
  std::vector<Vector2> areaThemPoints = {
      {theirGoalSides.first.x - margin, theirGoalSides.first.y - margin},
      {theirGoalSides.first.x + goalDepth, theirGoalSides.first.y - margin},
      {theirGoalSides.second.x + goalDepth, theirGoalSides.second.y + margin},
      {theirGoalSides.second.x - margin, theirGoalSides.second.y + margin}};

  interface::Input::drawDebugData(areaThemPoints, Qt::red, interface::Drawing::LINES_CONNECTED);

  return Polygon(areaThemPoints);
}

Polygon FieldComputations::getFieldEdge(double margin) {
    auto field = FieldMessage::get_field();

    double left = field[LEFTMOST_X] + margin;
    double right = field[RIGHTMOST_X] - margin;
    double bottom = field[BOTTOMMOST_Y] + margin;
    double top = field[TOPMOST_Y] - margin;

    std::vector<Vector2> fieldEdge = {
      {left, bottom},
      {left, top},
      {right, top},
      {right, bottom}};

    interface::Input::drawDebugData(fieldEdge, Qt::red, interface::Drawing::LINES_CONNECTED);

    return Polygon(fieldEdge);
}

} // world
} // ai
} // rtt
