//
// Created by Rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"


namespace rtt {
namespace ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsInDefenseArea::onUpdate() {
    Vector2 point;
    if (properties->getBool("useRobot")) {
        point = robot->pos;
    } else {
        point = ball->pos;
    }
    ourDefenseArea = properties->getBool("ourDefenseArea");
    outsideField = properties->getBool("outsideField");
    if (properties->hasDouble("margin")) margin = static_cast<float>(properties->getDouble("margin"));
    else margin = 0.0f;

    if (world::field->pointIsInDefenseArea(point, ourDefenseArea, margin, outsideField)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
