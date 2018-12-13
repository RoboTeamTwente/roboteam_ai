//
// Created by Rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"

namespace rtt {
namespace ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status IsInDefenseArea::update() {
    robot = getRobotFromProperties(properties);

    ourDefenseArea = properties->getBool("ourDefenseArea");
    if (properties->hasDouble("margin")) margin = static_cast<float>(properties->getDouble("margin"));
    else margin = 0.0f;

    roboteam_msgs::World world = World::get_world();

    if (Field::pointIsInDefenceArea(robot->pos, ourDefenseArea, margin)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
