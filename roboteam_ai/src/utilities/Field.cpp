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

/// returns the angle the goal points make from a point
double Field::getTotalGoalAngle(bool ourGoal, Vector2 point){
    std::pair<Vector2,Vector2> goal=getGoalSides(ourGoal);
    double AngleLeft=(goal.first-point).angle();
    double AngleRight=(goal.second-point).angle();
    return control::ControlUtils::angleDifference(control::ControlUtils::constrainAngle(AngleLeft),control::ControlUtils::constrainAngle(AngleRight));

}
double Field::getTotalVisibleGoalAngle(bool ourGoal, Vector2 point) {
    return getTotalGoalAngle(ourGoal,point)*getPercentageOfGoalVisibleFromPoint(ourGoal,point);
}
double Field::getPercentageOfGoalVisibleFromPoint(bool ourGoal, Vector2 point){
    auto field = Field::get_field();
    double goalWidth = field.goal_width;
    double blockadeLength = 0;
    for (auto const &blockade : getBlockadesMappedToGoal(ourGoal, point)) {
        blockadeLength += blockade.first.dist(blockade.second);
    }
    return std::max(100 - round(blockadeLength/goalWidth * 100), 0.0);
}

std::vector<std::pair<Vector2, Vector2>> Field::getBlockadesMappedToGoal(bool ourGoal, Vector2 point){
    const double robotRadius = Constants::ROBOT_RADIUS();

    Vector2 lowerGoalSide, upperGoalSide;
    lowerGoalSide = getGoalSides(ourGoal).first;
    upperGoalSide = getGoalSides(ourGoal).second;

    std::vector<std::pair<Vector2, Vector2>> blockades = {};

    // all the obstacles should be robots
    for (auto const &robot : World::getAllRobots()) {

        // discard already all robots that are not at all between the goal and point, or if a robot is standing on this point
        bool isRobotItself = point == robot.pos;
        bool isInPotentialBlockingZone = ourGoal ? robot.pos.x < point.x + robotRadius : robot.pos.x > point.x - robotRadius;
        if (!isRobotItself && isInPotentialBlockingZone) {

            // get the left and right sides of the robot
            auto lineToRobot = point - robot.pos;
            auto inverseLineToRobot = Vector2(-lineToRobot.y, lineToRobot.x);
            Vector2 upperSideOfRobot = inverseLineToRobot.stretchToLength(robotRadius) + robot.pos;
            Vector2 lowerSideOfRobot = inverseLineToRobot.stretchToLength(-robotRadius) + robot.pos;

            // map points onto goal line
            auto point1 = util::twoLineIntersection(point, lowerSideOfRobot, lowerGoalSide, upperGoalSide);
            auto point2 = util::twoLineIntersection(point, upperSideOfRobot, lowerGoalSide, upperGoalSide);

            // remove all obstacles that are completely out of the goal
            bool bothPointsBelowGoal = point1.y < lowerGoalSide.y && point2.y < lowerGoalSide.y;
            bool bothPointAboveGoal = point1.y > upperGoalSide.y && point2.y > upperGoalSide.y;
            if (!bothPointsBelowGoal && !bothPointAboveGoal) {

                // constrain the blockades to within the goal
                if (point1.y > point2.y) { // point1 is largest
                    point1.y = std::min(point1.y, upperGoalSide.y);
                    point2.y = std::max(point2.y, lowerGoalSide.y);
                    blockades.emplace_back(std::make_pair(point1, point2)); // the first element in the pair is the smallest
                } else { // point2 is largest
                    point2.y = std::min(point2.y, upperGoalSide.y);
                    point1.y = std::max(point1.y, lowerGoalSide.y);
                    blockades.emplace_back(std::make_pair(point2, point1)); // the first element in the pair is the smallest
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
std::vector<std::pair<Vector2, Vector2>> Field::mergeBlockades(std::vector<std::pair<Vector2, Vector2>> blockades) {
    std::vector<std::pair<Vector2, Vector2>> newBlockades;
    //remove 'duplicate' blockades
    for (auto Blockade=blockades.begin(); Blockade!=blockades.end(); ++ Blockade){
        bool duplicate=false;
        for (auto OtherBlockade=blockades.begin(); OtherBlockade!=blockades.end();++OtherBlockade){
            if (Blockade!=OtherBlockade){
                if(Blockade->first.y<OtherBlockade->first.y&&Blockade->second.y>OtherBlockade->second.y){
                    duplicate=true;
                    break;
                }
            }
        }
        if (!duplicate){
            newBlockades.push_back(*Blockade);
        }
    }
    blockades=newBlockades;
    // sort the blockades from low to high
    std::sort(blockades.begin(), blockades.end(), [](const std::pair<Vector2,Vector2> &a, const std::pair<Vector2,Vector2> &b) {
        return a.second.y < b.second.y;
    });

    std::vector<std::pair<Vector2, Vector2>> mergedBlockades;
    unsigned long iterator = 0;
    while (blockades.size() > (iterator + 1)) {
        if (blockades.at(iterator).first.y>blockades.at(iterator+1).second.y) {
            // if the first two elements intercept, merge them
            auto newBlockade = std::make_pair(blockades.at(iterator+1).first,blockades.at(iterator).second);
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
std::vector<std::pair<Vector2, Vector2>> Field::getVisiblePartsOfGoal(bool ourGoal, Vector2 point) {
    auto blockades = getBlockadesMappedToGoal(ourGoal, point);

    auto lower = getGoalSides(ourGoal).first;
    auto upper = getGoalSides(ourGoal).second;

    auto lowerHook = lower;
    std::vector<std::pair<Vector2, Vector2>> visibleParts = {};


    // we start from the lowerhook, which is the lowest goal side at the start.
    // The obstacles are sorted on their smallest value.
    // everytime we add a vector from the lowest goalside to the lowest part of the obstacle we remember the upper part of the obstacle
    // That upper part is stored as the lowerhook again: and we can repeat the process
    for (auto const &blockade : blockades) {
        auto lowerbound = std::min(blockade.first.y, blockade.second.y);

        // if the lowerbound is the same as the lower hook then the visible part has a length of 0 and we don't care about it
        if (lowerbound != lowerHook.y) {
            visibleParts.emplace_back(std::make_pair(lowerHook, Vector2(blockade.first.x, lowerbound)));
        }
        auto upperbound = std::max(blockade.first.y, blockade.second.y);
        lowerHook = Vector2(blockade.first.x, upperbound);
    }

    // if the last lowerhook is the same as the upper goal side then the visible part has a length of 0 and we don't care about it
    if (lowerHook != upper) {
        visibleParts.emplace_back(std::make_pair(lowerHook, upper));
    }
    return visibleParts;
}

// Returns the sides of the goal. The first vector is the the lower side and the second is the upper side.
std::pair<Vector2, Vector2> Field::getGoalSides(bool ourGoal) {
    roboteam_msgs::GeometryFieldSize _field;
    {
        std::lock_guard<std::mutex> lock(fieldMutex);
        _field = field;
    }

    // get the sides of the goal
    double goalWidth = _field.goal_width;
    auto goalCenter = ourGoal ? get_our_goal_center() : get_their_goal_center();
    Vector2 upperGoalSide = { goalCenter.x, goalCenter.y + (goalWidth/2)};
    Vector2 lowerGoalSide = { goalCenter.x, goalCenter.y - (goalWidth/2)};

    return std::make_pair(lowerGoalSide, upperGoalSide);
}

int Field::getRobotClosestToGoal(bool ourRobot, bool ourGoal) {
    roboteam_msgs::World_<std::allocator<void>>::_them_type robots = ourRobot ? World::get_world().us : World::get_world().them;
    Vector2 target = ourGoal ? Field::get_our_goal_center() : Field::get_their_goal_center();

    int closestId = World::getRobotClosestToPoint(robots, target)->id;
    return closestId;
}

} // ai
} // rtt
