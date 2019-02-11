//
// Created by mrlukasbos on 19-10-18.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "Field.h"

namespace rtt {
namespace ai {

using util = control::ControlUtils;

roboteam_msgs::GeometryFieldSize Field::field;
std::mutex Field::fieldMutex;

const roboteam_msgs::GeometryFieldSize Field::get_field() {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return Field::field;
}

void Field::set_field(roboteam_msgs::GeometryFieldSize field) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    Field::field = std::move(field);
}

Vector2 Field::get_our_goal_center() {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return Vector2(field.field_length/- 2, 0);
}

Vector2 Field::get_their_goal_center() {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return Vector2(field.field_length/2, 0);
}

bool Field::pointIsInDefenceArea(Vector2 point, bool isOurDefenceArea, float margin) {
    roboteam_msgs::GeometryFieldSize _field;
    {
        std::lock_guard<std::mutex> lock(fieldMutex);
        _field = field;
    }
    auto penaltyLine = isOurDefenceArea ? _field.left_penalty_line : _field.right_penalty_line;
    double yTopBound;
    double yBottomBound;
    double xBound = penaltyLine.begin.x;
    if (penaltyLine.begin.y < penaltyLine.end.y) {
        yBottomBound = penaltyLine.begin.y;
        yTopBound = penaltyLine.end.y;
    }
    else {
        yBottomBound = penaltyLine.end.y;
        yTopBound = penaltyLine.begin.y;
    }
    bool yIsWithinDefenceArea = point.y<(yTopBound + margin) && point.y>(yBottomBound - margin);
    bool xIsWithinOurDefenceArea = point.x < (xBound + margin);
    bool xIsWithinTheirDefenceArea = point.x > (xBound - margin);

    if (isOurDefenceArea){
        return xIsWithinOurDefenceArea&&yIsWithinDefenceArea;
    }
    else{
        return yIsWithinDefenceArea&&xIsWithinTheirDefenceArea;
    }
}

// the margin is pointed inside the field!
bool Field::pointIsInField(Vector2 point, float margin) {
    roboteam_msgs::GeometryFieldSize _field;
    {
        std::lock_guard<std::mutex> lock(fieldMutex);
        _field = field;
    }

    float halfLength = _field.field_length*0.5f;
    float halfWidth = _field.field_width*0.5f;

    return (point.x < halfLength - margin &&
            point.x > - halfLength + margin &&
            point.y < halfWidth - margin &&
            point.y > - halfWidth + margin);

}

double Field::getPercentageOfGoalVisibleFromPoint(bool ourGoal, Vector2 point){
    auto field = Field::get_field();
    double goalWidth = field.goal_width;

    double blockadeLength = 0;
    for (auto blockade : getBlockadesMappedToGoal(ourGoal, point)) {
        blockadeLength += blockade.first.dist(blockade.second);
    }

    return std::max(100 - round(blockadeLength/goalWidth * 100), 0.0);
}

std::vector<std::pair<Vector2, Vector2>> Field::getBlockadesMappedToGoal(bool ourGoal, Vector2 point){
    const double robotRadius = Constants::ROBOT_RADIUS();

    // get the sides of the goal
    auto field = Field::get_field();
    double goalWidth = field.goal_width;
    auto goalCenter = ourGoal ? get_our_goal_center() : get_their_goal_center();
    Vector2 leftGoalSide = { goalCenter.x, goalCenter.y + (goalWidth/2)};
    Vector2 rightGoalSide = { goalCenter.x, goalCenter.y - (goalWidth/2)};

    // vector to store the blockades0
    std::vector<std::pair<Vector2, Vector2>> blockades = {};

    // all the obstacles should be robots
    for (auto const &robot : World::getAllRobots()) {

        bool isRobotItself = point == robot.pos;
        bool isInPotentialBlockingZone = ourGoal ? robot.pos.x < point.x + robotRadius : robot.pos.x > point.x - robotRadius;

        // discard already all robots that are not at all between the goal and point, or if a robot is standing on this point
        if (!isRobotItself && isInPotentialBlockingZone) {

            // get the left and right sides of the robot
            auto lineToRobot = point-robot.pos;
            auto inversePointToRobot = Vector2(-lineToRobot.y, lineToRobot.x);
            Vector2 rightSideOfRobot = inversePointToRobot.stretchToLength(robotRadius)+robot.pos;
            Vector2 leftSideOfRobot = inversePointToRobot.stretchToLength(-robotRadius)+robot.pos;

            // get intersections from the line towards the sides of the robots with the goalline
            Vector2 leftPointOnGoalLine = util::twoLineIntersection(point, leftSideOfRobot, leftGoalSide,
                    rightGoalSide);
            Vector2 rightPointOnGoalLine = util::twoLineIntersection(point, rightSideOfRobot, leftGoalSide,
                    rightGoalSide);

            // if the right side, left side or center is in the triangle then we are quite sure there is a robot in the triangle.
            bool leftInTriangle = util::pointInTriangle(leftSideOfRobot, point, leftGoalSide, rightGoalSide);
            bool rightInTriangle = util::pointInTriangle(rightSideOfRobot, point, leftGoalSide, rightGoalSide);

            if (leftInTriangle || rightInTriangle) {
                auto left = leftInTriangle ? leftPointOnGoalLine : leftGoalSide;
                auto right = rightInTriangle ? rightPointOnGoalLine : rightGoalSide;
                blockades.emplace_back(std::make_pair(left, right));
            }
            else {

                /* edge case it fully blocks the view; both sides of the robot are out of sight though...
                 * we can instantly return that the view is fully blocked
                 */
                if (util::lineSegmentsIntersect(point, leftGoalSide, leftSideOfRobot, rightSideOfRobot)
                        && util::lineSegmentsIntersect(point, rightGoalSide, leftSideOfRobot, rightSideOfRobot)) {

                    return {std::make_pair(leftGoalSide, rightGoalSide)};
                };
                // else: this robot is not an obstacle
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
std::vector<std::pair<Vector2, Vector2>> Field::mergeBlockades(std::vector<std::pair<Vector2, Vector2>> blockades) {
    std::vector<std::pair<Vector2, Vector2>> mergedBlockades;
    unsigned long iterator = 0;
    while (blockades.size() > (iterator + 1)) {
        if (util::lineSegmentsIntersect(blockades.at(iterator).first, blockades.at(iterator).second,
                blockades.at(iterator + 1).first, blockades.at(iterator + 1).second)) {

            // if the first two elements intercept, merge them
            auto newBlockade = std::make_pair(blockades.at(iterator).first, blockades.at(iterator+1).second);
            blockades.erase(blockades.begin() + iterator + 1);
            blockades.at(iterator) = newBlockade;
        } else {
            iterator++;
        }
    }
    return blockades;
}

std::vector<std::pair<Vector2, Vector2>> Field::getVisiblePartsOfGoal(bool ourGoal, Vector2 point) {
    return std::vector<std::pair<Vector2, Vector2>>();
}

} // ai
} // rtt
