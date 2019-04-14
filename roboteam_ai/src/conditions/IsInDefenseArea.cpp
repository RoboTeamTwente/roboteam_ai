/*
* Returns true if the position is within the defence area.
* properties:
* - useRobot: if true, use the robot location, otherwise use the ball location
* - ourDefenceArea: if true, the condition returns true if the point is in our defence area. otherwise it is their defence area
* - outsideField: if true, the robot 
*/

#include "IsInDefenseArea.hpp"

namespace rtt {
namespace ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) 
: Condition(std::move(name), std::move(blackboard)) { }

void IsInDefenseArea::onInitialize() {
    point = properties->getBool("useRobot") ? robot->pos : ball->pos;
    ourDefenseArea = properties->getBool("ourDefenseArea");
    outsideField = properties->getBool("outsideField");
    margin = properties->hasDouble("margin") ? static_cast<float>(properties->getDouble("margin")) : 0.0f;
}

bt::Node::Status IsInDefenseArea::onUpdate() {
    if (world::field->pointIsInDefenceArea(point, ourDefenseArea, margin, outsideField)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
