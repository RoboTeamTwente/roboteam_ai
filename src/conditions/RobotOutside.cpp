//
// Created by baris on 1-5-19.
//

#include <world/FieldComputations.h>
#include <world/Robot.h>
#include "conditions/RobotOutside.h"

namespace rtt{
namespace ai{

RobotOutside::RobotOutside(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}
Condition::Status RobotOutside::onUpdate() {

    if (checkPoint()) {
        return Status::Success;
    }
    return Status::Failure;

}
bool RobotOutside::checkPoint() {
    // return success if the robot is out of the field
    // return success if the ball is out of the field
    FieldMessage field = FieldMessage::get_field();
    double margin = 0.15;
    return !(abs(robot->pos.x) < FieldMessage::get_field()[FIELD_LENGTH] / 2 + margin &&
        abs(robot->pos.y) < FieldMessage::get_field()[FIELD_WIDTH] / 2 + margin &&
        !FieldComputations::pointIsInDefenceArea(field, robot->pos));
}
}
}