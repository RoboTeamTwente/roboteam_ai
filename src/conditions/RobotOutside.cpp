//
// Created by baris on 1-5-19.
//

#include "conditions/RobotOutside.h"
#include <world/Field.h>
#include <world/Robot.h>

namespace rtt::ai {

RobotOutside::RobotOutside(std::string name, bt::Blackboard::Ptr blackboard)
    : Condition(name, blackboard) {}

Condition::Status RobotOutside::onUpdate() {
    if (checkPoint()) {
        return Status::Success;
    }
    return Status::Failure;
}

bool RobotOutside::checkPoint() {
    // return success if the robot is out of the field
    // return success if the ball is out of the field
    double margin = 0.15;
    return !(abs(robot->pos.x) < (*field).getFieldLength() / 2 + margin &&
        abs(robot->pos.y) < (*field).getFieldWidth() / 2 + margin &&
        !FieldComputations::pointIsInDefenceArea(*field, robot->pos));
}
}  // namespace rtt::ai