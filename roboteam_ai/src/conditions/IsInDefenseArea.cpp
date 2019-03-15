//
// Created by Rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryFieldSize.h"
#include "roboteam_utils/Vector2.h"
#include "../utilities/World.h"
#include "../utilities/Field.h"

namespace rtt {
namespace ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) { }

bt::Node::Status IsInDefenseArea::onUpdate() {
    Vector2 point;
    if (properties->getBool("useRobot")) {
        point = robot->pos;
    } else {
        point=ball->pos;
    }
    ourDefenseArea = properties->getBool("ourDefenseArea");
    if (properties->hasDouble("margin")) margin = static_cast<float>(properties->getDouble("margin"));
    else margin = 0.0f;

    roboteam_msgs::World world = World::get_world();
    if (Field::pointIsInDefenceArea(point, ourDefenseArea, margin)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
