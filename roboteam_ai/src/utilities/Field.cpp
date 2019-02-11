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
    return 0;
}

std::vector<std::pair<Vector2, Vector2>> Field::getBlockadesMappedToGoal(bool ourGoal, Vector2 point){
    const double robotRadius = Constants::ROBOT_RADIUS();
    auto field = Field::get_field();
    double goalWidth = field.goal_width;
    auto goalCenter = ourGoal ? get_our_goal_center() : get_their_goal_center();

    // get the sides of the goal
    Vector2 leftGoalSide = { goalCenter.x, goalCenter.y + (goalWidth/2)};
    Vector2 rightGoalSide = { goalCenter.x, goalCenter.y - (goalWidth/2)};

    std::vector<std::pair<Vector2, Vector2>> blockades = {};

    // all the obstacles should be robots
    for (auto const &robot : World::getAllRobots()) {

        // get the left and right sides of the robot
        Vector2 leftSideOfRobot = Vector2(robot.pos.x, robot.pos.y + robotRadius);
        Vector2 rightSideOfRobot = Vector2(robot.pos.x, robot.pos.y - robotRadius);

        // if the right side, left side or center is in the triangle then we are quite sure there is a robot in the triangle.
        bool leftInTriangle = util::pointInTriangle(leftSideOfRobot, point, leftGoalSide, rightGoalSide);
        bool rightInTriangle = util::pointInTriangle(rightSideOfRobot, point, leftGoalSide, rightGoalSide);
        bool intersectsLeftGoalLine = util::lineSegmentsIntersect(point, leftGoalSide, leftSideOfRobot,
                rightSideOfRobot);
        bool interSectsRightGoalLine = util::lineSegmentsIntersect(point, rightGoalSide, leftSideOfRobot,
                rightSideOfRobot);

        Vector2 leftPointOnGoalLine;
        Vector2 rightPointOnGoalLine;

        // the robot is fully in line of sight
        if (leftInTriangle && rightInTriangle) {
            leftPointOnGoalLine = util::twoLineIntersection(point, leftSideOfRobot, leftGoalSide, rightGoalSide);
            rightPointOnGoalLine = util::twoLineIntersection(point, rightSideOfRobot, leftGoalSide, rightGoalSide);
            blockades.emplace_back(std::make_pair(leftPointOnGoalLine, rightPointOnGoalLine));
        }

        // the left side of the robot is in sight
        else if (leftInTriangle) {
            leftPointOnGoalLine = util::twoLineIntersection(point, leftSideOfRobot, leftGoalSide, rightGoalSide);
            blockades.emplace_back(std::make_pair(leftPointOnGoalLine, rightGoalSide));
        }

        // the right side of the robot is in sight
        else if (rightInTriangle) {
            rightPointOnGoalLine = util::twoLineIntersection(point, rightSideOfRobot, leftGoalSide, rightGoalSide);
            blockades.emplace_back(std::make_pair(leftGoalSide, rightPointOnGoalLine));
        }

        // edge case it fully blocks the view; both sides of the robot are out of sight though...
        // we can instantly return that the view is fully blocked
        if (intersectsLeftGoalLine && interSectsRightGoalLine) return { std::make_pair(leftGoalSide, rightGoalSide) };

    }

    std::cout << "blockades before merge: " << blockades.size() << std::endl;


    // merge the blockades into one
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

    std::cout << "blockades after merge: " << blockades.size() << std::endl;

    double blockadeLength;
    for (auto blockade : blockades) {
        blockadeLength += blockade.first.dist(blockade.second);
    }

    std::cout << "percentage of goal blocked: " << round(blockadeLength/goalWidth * 100) << std::endl;
    return blockades;
}
std::vector<std::pair<Vector2, Vector2>> Field::getVisiblePartsOfGoal(bool ourGoal, Vector2 point) {
    auto field = Field::get_field();
    double goalWidth = field.goal_width;
    auto goalCenter = ourGoal ? get_our_goal_center() : get_their_goal_center();

    // get the sides of the goal
    Vector2 leftGoalSide = { goalCenter.x, goalCenter.y + (goalWidth/2)};
    Vector2 rightGoalSide = { goalCenter.x, goalCenter.y - (goalWidth/2)};



    // initialize the visible parts with a fully visible
    std::vector<std::pair<Vector2, Vector2>> visibleParts = {std::make_pair(leftGoalSide, rightGoalSide)};



    return std::vector<std::pair<Vector2, Vector2>>();
}
} // ai
} // rtt
