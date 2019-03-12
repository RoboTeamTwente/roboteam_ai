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

bt::Node::Status IsInDefenseArea::update() {
    Vector2 point;
    if (properties->getBool("useRobot")) {
        robot = getRobotFromProperties(properties);
        if (robot) {
            point = robot->pos;
        }
        else{
            return Status::Failure;
        }
    }
    else{
        auto ball=World::getBall();
        if (ball){
            point=ball->pos;
        }
        else{
            return Status::Failure;
        }

    }
    ourDefenseArea = properties->getBool("ourDefenseArea");
    outsideField = properties->getBool("outsideField");
    if (properties->hasDouble("margin")) margin = static_cast<float>(properties->getDouble("margin"));
    else margin = 0.0f;

    roboteam_msgs::World world = World::get_world();
    if (Field::pointIsInDefenceArea(point, ourDefenseArea, margin, outsideField)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
