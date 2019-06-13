/*
* Returns true if the position is within the defence area.
* properties:
* - useRobot: if true, use the robot location, otherwise use the ball location
* - ourDefenceArea: if true, the condition returns true if the point is in our defence area. otherwise it is their defence area
* - outsideField: if true, the robot 
*/

#include "roboteam_ai/src/world/Field.h"
#include "IsInDefenseArea.hpp"

namespace rtt {
namespace ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) 
: Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsInDefenseArea::onUpdate() {
    ourDefenseArea = properties->getBool("ourDefenseArea");
    outsideField = properties->getBool("outsideField");
    secondsAhead = properties->hasDouble("secondsAhead") ? properties->getDouble("secondsAhead") : 0.0;
    if(secondsAhead >= 0.0) {
        if(properties->getBool("useRobot")) {
            point = robot->pos + robot->vel * secondsAhead;
        } else {
            point = ball->pos + ball->vel * secondsAhead;
        }
    } else {
        point = properties->getBool("useRobot") ? robot->pos : ball->pos;
    }

    margin = properties->hasDouble("margin") ? static_cast<float>(properties->getDouble("margin")) : 0.0f;

    if (world::field->pointIsInDefenceArea(point, ourDefenseArea, margin, outsideField)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
