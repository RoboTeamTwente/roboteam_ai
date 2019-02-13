//
// Created by mrlukasbos on 19-10-18.
//

#include "Field.h"

namespace rtt {
namespace ai {

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

int Field::getRobotClosestToGoal(bool ourRobot, bool ourGoal) {
    roboteam_msgs::World_<std::allocator<void>>::_them_type robots = ourRobot ? World::get_world().us : World::get_world().them;
    Vector2 target = ourGoal ? Field::get_our_goal_center() : Field::get_their_goal_center();

    int closestId = World::getRobotClosestToPoint(robots, target)->id;
    return closestId;
}

} // ai
} // rtt
