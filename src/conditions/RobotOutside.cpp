//
// Created by baris on 1-5-19.
//

#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/world/Robot.h>
#include "include/roboteam_ai/conditions/RobotOutside.h"

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
    double margin=0.15;
    return ! (abs(robot->pos.x) < world::field->get_field().field_length/2 + margin&&
            abs(robot->pos.y) < world::field->get_field().field_width/2 + margin&&
            !world::field->pointIsInDefenceArea(robot->pos));
}
}
}